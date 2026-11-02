"""도메인 데이터 모델 및 불변 전송 객체(DTO) 정의 모듈."""

from dataclasses import dataclass

import numpy as np


@dataclass(slots=True, frozen=True)
class CameraIntrinsics:
    """카메라 내부 파라미터 행렬 및 렌즈 왜곡 계수 모델."""

    camera_matrix: np.ndarray  # 3x3 float64 행렬
    dist_coeffs: np.ndarray  # 5x1 또는 8x1 왜곡 계수 행렬


@dataclass(slots=True, frozen=True)
class TagCorner2D:
    """단일 AprilTag의 2D 화소 코너 좌표 및 식별 메타데이터."""

    tag_id: int
    corners: tuple[tuple[float, float], ...]  # 4개의 (x, y) 꼭짓점 좌표
    center: tuple[float, float]
    decision_margin: float


@dataclass(slots=True, frozen=True)
class Pose6DoF:
    """카메라 광학 좌표계 기준 6자유도 위치 및 자세 표현 모델."""

    tag_id: int
    translation: tuple[float, float, float]  # (x, y, z) 단위: 미터(m)
    quaternion: tuple[float, float, float, float]  # (qx, qy, qz, qw)
    rvec: np.ndarray  # 3x1 Rodrigues 회전 벡터
    tvec: np.ndarray  # 3x1 평행 이동 벡터
    reprojection_error: float  # 화소 단위 재투영 평균 오차 (px)
