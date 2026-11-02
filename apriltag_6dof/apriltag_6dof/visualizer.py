"""OpenCV 기반 AR 오버레이 및 3차원 축 시각화 렌더러 모듈."""

import cv2
import numpy as np

from apriltag_6dof.models import CameraIntrinsics, Pose6DoF, TagCorner2D


def render_ar_overlay(
    image: np.ndarray,
    tag: TagCorner2D,
    pose: Pose6DoF,
    intrinsics: CameraIntrinsics,
    tag_size: float,
) -> np.ndarray:
    """원본 BGR 이미지 위에 태그 바운딩 박스, 중심점, 3D 좌표축, 텍스트를 증강합니다."""
    annotated = image.copy()

    # 1. 2D 바운딩 박스 폴리라인(녹색) 및 중심점(적색) 드로잉
    corners_int = np.array(tag.corners, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(
        annotated, [corners_int], isClosed=True, color=(0, 255, 0), thickness=2
    )

    center_u, center_v = round(tag.center[0]), round(tag.center[1])
    cv2.circle(
        annotated, (center_u, center_v), radius=4, color=(0, 0, 255), thickness=-1
    )

    # 2. 마커 중심 기준 3차원 직교 좌표축(Frame Axes) 렌더링 (X:적, Y:녹, Z:청)
    axis_length = tag_size * 0.8
    cv2.drawFrameAxes(
        annotated,
        intrinsics.camera_matrix,
        intrinsics.dist_coeffs,
        pose.rvec,
        pose.tvec,
        axis_length,
        thickness=2,
    )

    # 3. 실시간 OSD(On-Screen Display) 상태 텍스트 출력
    dist_z = pose.translation[2]
    info_text = (
        f"ID:{pose.tag_id} | Z:{dist_z:.3f}m | Err:{pose.reprojection_error:.2f}px"
    )
    text_pos = (max(10, center_u - 70), max(25, center_v - 30))

    cv2.putText(
        annotated,
        info_text,
        text_pos,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    return annotated
