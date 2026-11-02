"""
Visualization builder for 2D OpenCV AR Overlays and RViz2 3D Markers.
Fully compliant with RViz OGRE character layout requirements.
"""

import cv2
import numpy as np
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from realsense_apriltag_rgbd_fusion.types import CameraIntrinsics, FusionOutput


class TagVisualizer:
    @staticmethod
    def draw_ar_overlay(
        image_bgr: np.ndarray,
        fused: FusionOutput,
        intrinsics: CameraIntrinsics,
        tag_size: float,
    ) -> None:
        # 1. 2D 검출 외곽선 다각형
        pts = fused.corners_2d.reshape((-1, 1, 2))
        cv2.polylines(image_bgr, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
        cv2.circle(
            image_bgr,
            (round(fused.u_c), round(fused.v_c)),
            radius=4,
            color=(0, 0, 255),
            thickness=-1,
        )

        # 2. 3D 좌표축 투영
        tvec_fused = np.array([[fused.x], [fused.y], [fused.z_depth]], dtype=np.float64)
        cv2.drawFrameAxes(
            image_bgr,
            intrinsics.matrix,
            intrinsics.dist_coeffs,
            fused.rvec,
            tvec_fused,
            length=tag_size * 0.8,
            thickness=2,
        )

        # 3. HUD 정보 텍스트
        u_idx = round(fused.u_c)
        v_idx = round(fused.v_c)
        info_text = f"ID:{fused.tag_id} | Z:{fused.z_depth:.3f}m | Res:{fused.residual_m * 1000:.1f}mm"
        cv2.putText(
            image_bgr,
            info_text,
            (u_idx - 60, max(20, v_idx - 35)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    @staticmethod
    def create_rviz_markers(
        header: Header,
        fused: FusionOutput,
        tag_size: float,
    ) -> MarkerArray:
        # 마커 1: 3D 노란색 마커 평면 플레이트
        cube_marker = Marker()
        cube_marker.header = header
        cube_marker.ns = "tag_bounding_box"
        cube_marker.id = fused.tag_id
        cube_marker.type = Marker.CUBE
        cube_marker.action = Marker.ADD
        cube_marker.pose.position.x = fused.x
        cube_marker.pose.position.y = fused.y
        cube_marker.pose.position.z = fused.z_depth
        cube_marker.pose.orientation.x = fused.quaternion[0]
        cube_marker.pose.orientation.y = fused.quaternion[1]
        cube_marker.pose.orientation.z = fused.quaternion[2]
        cube_marker.pose.orientation.w = fused.quaternion[3]
        cube_marker.scale.x = tag_size
        cube_marker.scale.y = tag_size
        cube_marker.scale.z = 0.002
        cube_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7)  # 노란색 플레이트
        cube_marker.lifetime.sec = 0
        cube_marker.lifetime.nanosec = 250000000

        # 마커 2: 3D 텍스트 라벨 (OGRE 자간 폭발 버그 방지)
        text_marker = Marker()
        text_marker.header = header
        text_marker.ns = "tag_text"
        text_marker.id = fused.tag_id + 1000
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = fused.x
        text_marker.pose.position.y = fused.y - (tag_size * 0.55)
        text_marker.pose.position.z = fused.z_depth
        text_marker.pose.orientation = cube_marker.pose.orientation

        font_size = 0.016
        text_marker.scale.x = font_size
        text_marker.scale.y = font_size
        text_marker.scale.z = font_size
        text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text_marker.text = f"Tag{fused.tag_id}:Z={fused.z_depth:.3f}m"
        text_marker.lifetime.sec = 0
        text_marker.lifetime.nanosec = 250000000

        array = MarkerArray()
        array.markers = [cube_marker, text_marker]
        return array
