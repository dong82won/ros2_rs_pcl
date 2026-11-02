"""
Pure Computer Vision and Mathematical Estimation Engine for RGB-D AprilTag Fusion.
Decoupled entirely from ROS middleware for deterministic unit testability.
"""

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

from realsense_apriltag_rgbd_fusion.types import CameraIntrinsics, FusionOutput


class RgbdTagFusionEngine:
    def __init__(
        self,
        tag_size: float = 0.048,
        residual_threshold_m: float = 0.030,
        roi_half_size: int = 2,
    ) -> None:
        self.tag_size: float = tag_size
        self.residual_threshold_m: float = residual_threshold_m
        self.roi_half_size: int = roi_half_size

    def set_parameters(
        self,
        tag_size: float | None = None,
        residual_threshold_m: float | None = None,
        roi_half_size: int | None = None,
    ) -> None:
        if tag_size is not None:
            self.tag_size = tag_size
        if residual_threshold_m is not None:
            self.residual_threshold_m = residual_threshold_m
        if roi_half_size is not None:
            self.roi_half_size = roi_half_size

    def extract_median_depth(
        self, depth_image: np.ndarray, u_c: float, v_c: float
    ) -> float | None:
        img_h, img_w = depth_image.shape
        u_idx, v_idx = round(u_c), round(v_c)
        k = self.roi_half_size

        u_min, u_max = max(0, u_idx - k), min(img_w, u_idx + k + 1)
        v_min, v_max = max(0, v_idx - k), min(img_h, v_idx + k + 1)

        roi = depth_image[v_min:v_max, u_min:u_max]
        valid_depths = roi[roi > 0]

        if valid_depths.size == 0:
            return None

        # 밀리미터(16UC1) -> 미터 단위 변환
        return float(np.median(valid_depths)) * 0.001

    def solve_pnp(
        self, corners_2d: np.ndarray, intrinsics: CameraIntrinsics
    ) -> tuple[bool, np.ndarray | None, np.ndarray | None]:
        s = self.tag_size / 2.0
        obj_points = np.array(
            [
                [-s, s, 0.0],
                [s, s, 0.0],
                [s, -s, 0.0],
                [-s, -s, 0.0],
            ],
            dtype=np.float64,
        )

        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            corners_2d,
            intrinsics.matrix,
            intrinsics.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not success:
            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                corners_2d,
                intrinsics.matrix,
                intrinsics.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

        return success, rvec, tvec

    def process(
        self,
        tag_id: int,
        raw_corners: list,
        depth_image: np.ndarray,
        intrinsics: CameraIntrinsics,
    ) -> tuple[bool, FusionOutput | None, str]:
        # 1. 2D 화소 중심 연산
        corners_np = np.array(
            [[float(c.x), float(c.y)] for c in raw_corners], dtype=np.float64
        )
        u_c = float(np.mean(corners_np[:, 0]))
        v_c = float(np.mean(corners_np[:, 1]))

        # 2. ROI Depth 중앙값 추출
        z_depth = self.extract_median_depth(depth_image, u_c, v_c)
        if z_depth is None:
            return False, None, "ROI Depth 유효 측정치 부재 (Min-Z 거리 제약)"

        # 3. PnP 6-DoF 솔빙
        pnp_ok, rvec, tvec = self.solve_pnp(corners_np, intrinsics)
        if not pnp_ok or rvec is None or tvec is None:
            return False, None, "PnP 솔버 수렴 실패"

        z_pnp = float(tvec[2, 0])
        residual = abs(z_pnp - z_depth)

        # 4. 잔차 게이트 검증
        if residual > self.residual_threshold_m:
            return (
                False,
                None,
                f"잔차 임계값 초과 ({residual * 1000:.1f}mm > {self.residual_threshold_m * 1000:.1f}mm)",
            )

        # 5. Intrinsic 3차원 Metric 역투영
        x_final = ((u_c - intrinsics.cx) * z_depth) / intrinsics.fx
        y_final = ((v_c - intrinsics.cy) * z_depth) / intrinsics.fy
        z_final = z_depth

        # 6. 회전 행렬 쿼터니언 변환
        rot_mat, _ = cv2.Rodrigues(rvec)
        rot_scipy = R_scipy.from_matrix(rot_mat)
        q = rot_scipy.as_quat(canonical=False)
        quat_tuple = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))

        output = FusionOutput(
            tag_id=tag_id,
            u_c=u_c,
            v_c=v_c,
            x=x_final,
            y=y_final,
            z_depth=z_final,
            z_pnp=z_pnp,
            residual_m=residual,
            quaternion=quat_tuple,
            rvec=rvec,
            tvec=tvec,
            corners_2d=np.round(corners_np).astype(np.int32),
        )

        return True, output, "정상 추적 완료"
