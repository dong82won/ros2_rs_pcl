"""2.5D Depth Culling 및 RANSAC 듀얼 평면 피팅 기반 레버 6-DoF 상태 추정 모듈."""

import math
from dataclasses import dataclass

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from apriltag_6dof.models import CameraIntrinsics


@dataclass(slots=True, frozen=True)
class VisualPlaneParams:
    """기하 연산 및 필터링 제어 파라미터 구조체."""

    handle_nominal_offset_x: float = -0.065
    handle_nominal_offset_y: float = -0.110
    roi_search_padding_px: int = 15
    min_contour_area_px: int = 600
    min_aspect_ratio: float = 2.2

    # [추가됨] 2.5D Depth Culling 기준값: 도어 표면 기준 레버의 최소/최대 돌출 허용치
    min_protrusion_filter_m: float = (
        0.010  # 배전반 표면(0mm) 및 틈새 노이즈(10mm 이하) 완벽 배제
    )
    max_protrusion_filter_m: float = 0.080  # 비정상적으로 튀는 전경 노이즈 배제

    ransac_max_iterations: int = 80
    ransac_distance_threshold_m: float = 0.004
    ransac_min_inliers: int = 40  # 필터링 후 점군이 줄어들 것을 대비해 50 -> 40 완화
    filter_alpha: float = 0.25
    protrusion_unlock_thresh: float = 0.038
    protrusion_lock_thresh: float = 0.030
    tilt_unlock_thresh_deg: float = 4.5
    tilt_lock_thresh_deg: float = 2.5
    state_debounce_frames: int = 4


@dataclass(slots=True, frozen=True)
class LeverPoseOutput:
    """파이프라인 최종 출력 데이터 구조체."""

    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    tilt_deg: float
    protrusion_m: float
    z_door_m: float
    is_opened: bool
    tracking_valid: bool
    handle_poly_2d: np.ndarray


class VisualPlaneKinematicsEngine:
    """2.5D 점군 사전 컬링과 RANSAC 3D 평면 피팅을 결합한 6-DoF 실시간 상태 추정 엔진."""

    def __init__(self, params: VisualPlaneParams) -> None:
        self._params = params
        self._f_pos: np.ndarray | None = None
        self._f_quat: np.ndarray | None = None

        self._is_opened: bool = False
        self._unlock_votes: int = 0
        self._lock_votes: int = 0

    def reset_filter(self) -> None:
        self._f_pos = None
        self._f_quat = None
        self._is_opened = False
        self._unlock_votes = 0
        self._lock_votes = 0

    def _get_coarse_roi_window(
        self,
        rvec: np.ndarray,
        tvec: np.ndarray,
        cam_mat: np.ndarray,
        dist_coeffs: np.ndarray,
        img_w: int,
        img_h: int,
    ) -> tuple[int, int, int, int]:
        cx = self._params.handle_nominal_offset_x
        cy = self._params.handle_nominal_offset_y

        coarse_corners_3d = np.array(
            [
                [cx - 0.025, cy - 0.160, 0.0],
                [cx + 0.025, cy - 0.160, 0.0],
                [cx + 0.025, cy + 0.160, 0.0],
                [cx - 0.025, cy + 0.160, 0.0],
            ],
            dtype=np.float64,
        )

        proj_pts, _ = cv2.projectPoints(
            coarse_corners_3d, rvec, tvec, cam_mat, dist_coeffs
        )
        pts_2d = proj_pts.reshape(-1, 2)

        pad = self._params.roi_search_padding_px
        u_min = max(0, int(np.min(pts_2d[:, 0])) - pad)
        u_max = min(img_w - 1, int(np.max(pts_2d[:, 0])) + pad)
        v_min = max(0, int(np.min(pts_2d[:, 1])) - pad)
        v_max = min(img_h - 1, int(np.max(pts_2d[:, 1])) + pad)

        return u_min, v_min, u_max, v_max

    def _lock_2d_contour(
        self,
        cv_image: np.ndarray,
        u_min: int,
        v_min: int,
        u_max: int,
        v_max: int,
    ) -> np.ndarray | None:
        roi_bgr = cv_image[v_min:v_max, u_min:u_max]
        if roi_bgr.shape[0] < 30 or roi_bgr.shape[1] < 20:
            return None

        h_roi, w_roi = roi_bgr.shape[:2]
        roi_center_x = w_roi / 2.0
        roi_center_y = h_roi / 2.0

        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        filtered = cv2.bilateralFilter(gray, d=7, sigmaColor=50, sigmaSpace=50)

        kernel_v = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 9))
        morph_grad = cv2.morphologyEx(filtered, cv2.MORPH_GRADIENT, kernel_v)

        _, thresh = cv2.threshold(
            morph_grad, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel_v, iterations=2)

        contours, _ = cv2.findContours(
            closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None

        best_poly = None
        max_score = -1.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self._params.min_contour_area_px:
                continue

            rect = cv2.minAreaRect(cnt)
            w_box, h_box = rect[1]
            if w_box < 1e-4 or h_box < 1e-4:
                continue

            aspect_ratio = max(w_box, h_box) / min(w_box, h_box)
            if aspect_ratio < self._params.min_aspect_ratio:
                continue

            box = cv2.boxPoints(rect)
            box_int = np.asarray(box, dtype=np.int32)

            M = cv2.moments(cnt)
            if M["m00"] > 0:
                c_x = M["m10"] / M["m00"]
                c_y = M["m01"] / M["m00"]
            else:
                c_x, c_y = rect[0]

            dist_from_center = math.hypot(c_x - roi_center_x, c_y - roi_center_y)
            dist_penalty = 1.0 + (dist_from_center / max(w_roi, 1.0)) * 5.0

            score = (area * aspect_ratio) / dist_penalty

            if score > max_score:
                max_score = score
                best_poly = box_int

        if best_poly is not None:
            best_poly[:, 0] += u_min
            best_poly[:, 1] += v_min
            return best_poly

        return None

    def _fit_plane_ransac(
        self,
        points: np.ndarray,
    ) -> tuple[np.ndarray, float, np.ndarray] | None:
        num_points = points.shape[0]
        if num_points < self._params.ransac_min_inliers:
            return None

        best_inliers = np.array([], dtype=np.int64)
        thresh = self._params.ransac_distance_threshold_m

        for _ in range(self._params.ransac_max_iterations):
            idx = np.random.choice(num_points, 3, replace=False)
            p1, p2, p3 = points[idx]

            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal /= norm
            d = -float(np.dot(normal, p1))

            distances = np.abs(np.dot(points, normal) + d)
            inlier_indices = np.where(distances < thresh)[0]

            if len(inlier_indices) > len(best_inliers):
                best_inliers = inlier_indices

        if len(best_inliers) < self._params.ransac_min_inliers:
            return None

        inlier_pts = points[best_inliers]
        if inlier_pts.shape[0] < 3:
            return None

        centroid = np.mean(inlier_pts, axis=0)
        _, _, vh = np.linalg.svd(inlier_pts - centroid)
        refined_normal = vh[2, :]
        refined_normal /= np.linalg.norm(refined_normal)

        if refined_normal[2] > 0.0:
            refined_normal = -refined_normal

        refined_d = -float(np.dot(refined_normal, centroid))
        return refined_normal, refined_d, inlier_pts

    def process(
        self,
        rvec: np.ndarray,
        tvec: np.ndarray,
        depth_m: np.ndarray,
        cv_image: np.ndarray,
        intrinsics: CameraIntrinsics,
    ) -> LeverPoseOutput:
        img_h, img_w = depth_m.shape[:2]
        cam_mat = intrinsics.camera_matrix
        dist_coeffs = intrinsics.dist_coeffs

        rot_mat, _ = cv2.Rodrigues(rvec)
        # 도어 평면 방정식 계산 (Tag Z축 기준)
        n_door = -rot_mat[:, 2]
        n_door /= np.linalg.norm(n_door)
        p_door_tag = tvec.ravel()
        d_door = -float(np.dot(n_door, p_door_tag))

        u_min, v_min, u_max, v_max = self._get_coarse_roi_window(
            rvec, tvec, cam_mat, dist_coeffs, img_w, img_h
        )

        locked_poly = self._lock_2d_contour(cv_image, u_min, v_min, u_max, v_max)

        if locked_poly is None:
            corners_nominal_3d = np.array(
                [
                    [
                        self._params.handle_nominal_offset_x - 0.023,
                        self._params.handle_nominal_offset_y - 0.160,
                        0.0,
                    ],
                    [
                        self._params.handle_nominal_offset_x + 0.023,
                        self._params.handle_nominal_offset_y - 0.160,
                        0.0,
                    ],
                    [
                        self._params.handle_nominal_offset_x + 0.023,
                        self._params.handle_nominal_offset_y + 0.160,
                        0.0,
                    ],
                    [
                        self._params.handle_nominal_offset_x - 0.023,
                        self._params.handle_nominal_offset_y + 0.160,
                        0.0,
                    ],
                ],
                dtype=np.float64,
            )
            proj_pts, _ = cv2.projectPoints(
                corners_nominal_3d, rvec, tvec, cam_mat, dist_coeffs
            )
            locked_poly = proj_pts.reshape(-1, 2).astype(np.int32)
            locked_poly[:, 0] = np.clip(locked_poly[:, 0], 0, img_w - 1)
            locked_poly[:, 1] = np.clip(locked_poly[:, 1], 0, img_h - 1)

        poly_mask = np.zeros((img_h, img_w), dtype=np.uint8)
        cv2.fillPoly(poly_mask, [locked_poly], 255)

        v_indices, u_indices = np.where(poly_mask > 0)
        depths = depth_m[v_indices, u_indices]

        valid_idx = np.where(np.isfinite(depths) & (depths > 0.3) & (depths < 2.0))[0]
        u_val = u_indices[valid_idx]
        v_val = v_indices[valid_idx]
        z_val = depths[valid_idx]

        fx = float(cam_mat[0, 0])
        fy = float(cam_mat[1, 1])
        ppx = float(cam_mat[0, 2])
        ppy = float(cam_mat[1, 2])

        x_pts = (u_val - ppx) * z_val / fx
        y_pts = (v_val - ppy) * z_val / fy
        points_3d = np.column_stack([x_pts, y_pts, z_val])

        # =========================================================================
        # [핵심 수정] 2.5D Culling: 도어 평면 방정식(n_door, d_door)을 이용해
        # 배전반 배경 픽셀(도어 틈새 포함)을 RANSAC 투입 전 완벽히 제거합니다.
        # =========================================================================
        if points_3d.shape[0] > 0:
            # 부호 있는 점-평면 거리 산출
            dist_to_door = np.dot(points_3d, n_door) + d_door
            # 10mm ~ 80mm 사이로 튀어나온 점(순수 레버)만 남김
            culling_mask = (dist_to_door > self._params.min_protrusion_filter_m) & (
                dist_to_door < self._params.max_protrusion_filter_m
            )
            lever_points_3d = points_3d[culling_mask]
        else:
            lever_points_3d = np.empty((0, 3))

        ransac_result = self._fit_plane_ransac(lever_points_3d)

        if ransac_result is not None:
            n_lever, _, inlier_pts = ransac_result
            centroid_lever = np.mean(inlier_pts, axis=0)

            actual_protrusion_m = float(np.abs(np.dot(n_door, centroid_lever) + d_door))

            cos_tilt = float(np.dot(n_door, n_lever))
            cos_tilt = min(1.0, max(-1.0, cos_tilt))
            tilt_deg = math.degrees(math.acos(cos_tilt))

            cov = np.cov(inlier_pts - centroid_lever, rowvar=False)
            eigenvals, eigenvecs = np.linalg.eigh(cov)
            major_axis = eigenvecs[:, np.argmax(eigenvals)]
            if major_axis[1] < 0.0:
                major_axis = -major_axis

            z_tool = -n_lever
            z_tool /= np.linalg.norm(z_tool)
            y_tool = major_axis - np.dot(major_axis, z_tool) * z_tool
            y_tool /= np.linalg.norm(y_tool)
            x_tool = np.cross(y_tool, z_tool)
            x_tool /= np.linalg.norm(x_tool)

            rot_target = R.from_matrix(np.column_stack([x_tool, y_tool, z_tool]))
            raw_quat = np.asarray(rot_target.as_quat(canonical=False), dtype=np.float64)
            raw_pos = centroid_lever
        else:
            centroid_lever = p_door_tag + rot_mat @ np.array(
                [
                    self._params.handle_nominal_offset_x,
                    self._params.handle_nominal_offset_y,
                    -0.025,
                ]
            )
            raw_pos = centroid_lever
            raw_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
            actual_protrusion_m = 0.025
            tilt_deg = 0.0

        z_door = float(p_door_tag[2])

        is_unlock_event = (
            actual_protrusion_m >= self._params.protrusion_unlock_thresh
            or tilt_deg >= self._params.tilt_unlock_thresh_deg
        )
        is_lock_event = (
            actual_protrusion_m <= self._params.protrusion_lock_thresh
            and tilt_deg <= self._params.tilt_lock_thresh_deg
        )

        if is_unlock_event:
            self._unlock_votes += 1
            self._lock_votes = 0
            if self._unlock_votes >= self._params.state_debounce_frames:
                self._is_opened = True
        elif is_lock_event:
            self._lock_votes += 1
            self._unlock_votes = 0
            if self._lock_votes >= self._params.state_debounce_frames:
                self._is_opened = False
        else:
            self._unlock_votes = 0
            self._lock_votes = 0

        alpha = self._params.filter_alpha
        if self._f_pos is None or self._f_quat is None:
            filtered_pos = raw_pos.copy()
            filtered_quat = raw_quat.copy()
        else:
            filtered_pos = alpha * raw_pos + (1.0 - alpha) * self._f_pos
            aligned_quat = raw_quat.copy()
            if float(np.dot(self._f_quat, aligned_quat)) < 0.0:
                aligned_quat = -aligned_quat
            interp_quat = (1.0 - alpha) * self._f_quat + alpha * aligned_quat
            norm_val = float(np.linalg.norm(interp_quat))
            filtered_quat = interp_quat / norm_val if norm_val > 1e-6 else aligned_quat

        self._f_pos = filtered_pos
        self._f_quat = filtered_quat
        f_euler = R.from_quat(filtered_quat).as_euler("xyz", degrees=True)

        return LeverPoseOutput(
            x=float(filtered_pos[0]),
            y=float(filtered_pos[1]),
            z=float(filtered_pos[2]),
            qx=float(filtered_quat[0]),
            qy=float(filtered_quat[1]),
            qz=float(filtered_quat[2]),
            qw=float(filtered_quat[3]),
            roll_deg=float(f_euler[0]),
            pitch_deg=float(f_euler[1]),
            yaw_deg=float(f_euler[2]),
            tilt_deg=tilt_deg,
            protrusion_m=actual_protrusion_m,
            z_door_m=z_door,
            is_opened=self._is_opened,
            tracking_valid=True,
            handle_poly_2d=locked_poly,
        )


LeverKinematicsEngine = VisualPlaneKinematicsEngine
