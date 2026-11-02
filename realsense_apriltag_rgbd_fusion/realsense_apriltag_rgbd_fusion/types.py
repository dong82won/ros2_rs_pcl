"""
Data transfer objects and type specifications for AprilTag RGB-D fusion pipeline.
Compliant with Modern Python 3.12+ strict typing standards.
"""

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True, slots=True)
class CameraIntrinsics:
    matrix: np.ndarray  # 3x3 float64
    dist_coeffs: np.ndarray  # 5x1 float64
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_params(
        cls,
        k: Sequence[float] | np.ndarray,
        d: Sequence[float] | np.ndarray,
    ) -> "CameraIntrinsics":
        """
        ROS 2 CameraInfo 메시지의 k, d 필드를 타입 안전하게 파싱하여 인트린식 객체 생성.
        array.array, list, tuple, numpy.ndarray를 모두 지원.
        """
        mat = np.array(k, dtype=np.float64).reshape((3, 3))

        # 왜곡 계수 차원 정규화 (최소 5개 계수 확보)
        if len(d) >= 5:
            dist = np.array(d[:5], dtype=np.float64).reshape((5, 1))
        elif len(d) > 0:
            padded = np.zeros(5, dtype=np.float64)
            padded[: len(d)] = np.array(d, dtype=np.float64)
            dist = padded.reshape((5, 1))
        else:
            dist = np.zeros((5, 1), dtype=np.float64)

        return cls(
            matrix=mat,
            dist_coeffs=dist,
            fx=float(mat[0, 0]),
            fy=float(mat[1, 1]),
            cx=float(mat[0, 2]),
            cy=float(mat[1, 2]),
        )


@dataclass(frozen=True, slots=True)
class FusionOutput:
    tag_id: int
    u_c: float
    v_c: float
    x: float
    y: float
    z_depth: float
    z_pnp: float
    residual_m: float
    quaternion: tuple[float, float, float, float]  # (x, y, z, w)
    rvec: np.ndarray
    tvec: np.ndarray
    corners_2d: np.ndarray  # 4x2 int32
