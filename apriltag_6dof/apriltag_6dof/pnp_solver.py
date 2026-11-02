"""OpenCV 기반 3D 원근 기하(PnP) 연산 및 쿼터니언 변환 모듈."""

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from apriltag_6dof.models import CameraIntrinsics, Pose6DoF, TagCorner2D


def compute_reprojection_error(
    obj_points: np.ndarray,
    img_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    intrinsics: CameraIntrinsics,
) -> float:
    """계산된 3D 포즈를 2D 화소로 재투영하여 기하학적 오차(픽셀)를 산출합니다."""
    projected, _ = cv2.projectPoints(
        obj_points,
        rvec,
        tvec,
        intrinsics.camera_matrix,
        intrinsics.dist_coeffs,
    )
    diff = img_points - projected.reshape(-1, 2)
    return float(np.mean(np.linalg.norm(diff, axis=1)))


def solve_tag_pnp(
    tag: TagCorner2D,
    tag_size: float,
    intrinsics: CameraIntrinsics,
) -> Pose6DoF | None:
    """2D 태그 코너와 카메라 내부 파라미터를 이용해 6-DoF 위치 및 자세를 계산합니다."""
    if len(tag.corners) != 4:
        return None

    half_s = tag_size / 2.0

    # apriltag_ros 검출 코너(0:좌하, 1:우하, 2:우상, 3:좌상)와 1:1 일치하는 3D 모델
    obj_points = np.array(
        [
            [-half_s, half_s, 0.0],
            [half_s, half_s, 0.0],
            [half_s, -half_s, 0.0],
            [-half_s, -half_s, 0.0],
        ],
        dtype=np.float64,
    )

    img_points = np.array(tag.corners, dtype=np.float64)

    # 1. 평면 사각 마커 전용 고속/고정밀 IPPE_SQUARE 솔버 적용
    success, rvec, tvec = cv2.solvePnP(
        obj_points,
        img_points,
        intrinsics.camera_matrix,
        intrinsics.dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )

    # 수렴 실패 시 일반 반복 최적화(Levenberg-Marquardt) 방식으로 폴백
    if not success or rvec is None or tvec is None:
        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            intrinsics.camera_matrix,
            intrinsics.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

    if not success or rvec is None or tvec is None:
        return None

    # 2. 품질 검증을 위한 재투영 오차 산출
    reproj_err = compute_reprojection_error(
        obj_points,
        img_points,
        rvec,
        tvec,
        intrinsics,
    )

    # 3. Rodrigues 회전 벡터 -> SciPy 쿼터니언 (x, y, z, w) 직접 변환
    # 불필요한 인위적 회전(180도 오프셋)을 제거하여 AR 영상의 rvec과 TF의 quat을 100% 일치시킵니다.
    rot = Rotation.from_rotvec(rvec.ravel())
    quat = rot.as_quat(canonical=False)

    return Pose6DoF(
        tag_id=tag.tag_id,
        translation=(float(tvec[0, 0]), float(tvec[1, 0]), float(tvec[2, 0])),
        quaternion=(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
        rvec=rvec,
        tvec=tvec,
        reprojection_error=reproj_err,
    )
