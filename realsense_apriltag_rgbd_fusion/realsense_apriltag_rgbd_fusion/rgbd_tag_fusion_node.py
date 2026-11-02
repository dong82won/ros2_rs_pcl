"""
ROS 2 Jazzy Jalisco - High-Precision RGB-D AprilTag Fusion Node.
Validated for Python 3.12+, Modern Typing, Pylance, and Strict Standards.
"""

import cv2
import message_filters
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, TransformStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R_scipy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class RgbdTagFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("rgbd_tag_fusion_node")

        # 1. Dynamic Parameter Declarations
        self.declare_parameter("target_tag_id", 0)
        self.declare_parameter("tag_size", 0.048)  # 실측 규격 4.8cm 반영
        self.declare_parameter("target_frame", "camera_color_optical_frame")
        self.declare_parameter("residual_threshold_m", 0.030)  # 정밀 잔차 3cm 게이트
        self.declare_parameter("min_decision_margin", 15.0)
        self.declare_parameter("depth_roi_half_size", 2)
        self.declare_parameter("sync_slop_sec", 0.060)

        self._target_tag_id: int = (
            self.get_parameter("target_tag_id").get_parameter_value().integer_value
        )
        self._tag_size: float = (
            self.get_parameter("tag_size").get_parameter_value().double_value
        )
        self._target_frame: str = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self._residual_threshold_m: float = (
            self.get_parameter("residual_threshold_m")
            .get_parameter_value()
            .double_value
        )
        self._min_decision_margin: float = (
            self.get_parameter("min_decision_margin").get_parameter_value().double_value
        )
        self._depth_roi_half_size: int = (
            self.get_parameter("depth_roi_half_size")
            .get_parameter_value()
            .integer_value
        )
        self._sync_slop_sec: float = (
            self.get_parameter("sync_slop_sec").get_parameter_value().double_value
        )

        self.add_on_set_parameters_callback(self._on_parameter_change)

        # 2. Internal Buffers and ROS Interfaces
        self._camera_matrix: np.ndarray | None = None
        self._dist_coeffs: np.ndarray = np.zeros((5, 1), dtype=np.float64)
        self._cv_bridge: CvBridge = CvBridge()
        self._tf_broadcaster: TransformBroadcaster = TransformBroadcaster(self)
        self._cb_group: ReentrantCallbackGroup = ReentrantCallbackGroup()
        self._publish_count: int = 0

        # 3. Deterministic QoS Profiles
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 4. Camera Intrinsics Subscriber
        self._sub_camera_info = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self._camera_info_callback,
            sensor_qos,
            callback_group=self._cb_group,
        )

        # 5. Triple ApproximateTime Synchronizer Pipeline
        self._sub_color = message_filters.Subscriber(
            self,
            Image,
            "/camera/camera/color/image_raw",
            qos_profile=sensor_qos,
            callback_group=self._cb_group,
        )

        self._sub_depth = message_filters.Subscriber(
            self,
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            qos_profile=sensor_qos,
            callback_group=self._cb_group,
        )

        self._sub_detections = message_filters.Subscriber(
            self,
            AprilTagDetectionArray,
            "/detections",
            qos_profile=sensor_qos,
            callback_group=self._cb_group,
        )

        self._time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self._sub_color, self._sub_depth, self._sub_detections],
            queue_size=20,
            slop=self._sync_slop_sec,
        )
        self._time_synchronizer.registerCallback(self._synchronized_callback)

        # 6. Production Publishers
        self._pub_target_pose = self.create_publisher(
            PoseStamped, "~/target_tag_pose", reliable_qos
        )
        self._pub_annotated_image = self.create_publisher(
            Image, "~/annotated_image", sensor_qos
        )
        self._pub_markers = self.create_publisher(
            MarkerArray, "~/target_tag_markers", reliable_qos
        )

        self.get_logger().info(
            f"RgbdTagFusionNode 초기화 완료. Target ID: {self._target_tag_id}, "
            f"규격: {self._tag_size * 1000:.1f}mm, 허용 잔차: {self._residual_threshold_m * 1000:.1f}mm"
        )

    def _on_parameter_change(self, params: list[Parameter]) -> SetParametersResult:
        result = SetParametersResult(successful=True, reason="파라미터 변경 적용 완료")
        for param in params:
            if param.name == "target_tag_id" and param.value is not None:
                self._target_tag_id = int(param.value)
            elif param.name == "tag_size" and param.value is not None:
                self._tag_size = float(param.value)
            elif param.name == "residual_threshold_m" and param.value is not None:
                self._residual_threshold_m = float(param.value)
            elif param.name == "min_decision_margin" and param.value is not None:
                self._min_decision_margin = float(param.value)
        return result

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            if len(msg.d) >= 5:
                self._dist_coeffs = np.array(msg.d[:5], dtype=np.float64).reshape(
                    (5, 1)
                )
            self.get_logger().info(
                f"카메라 Intrinsic 로드 완료: fx={self._camera_matrix[0, 0]:.2f}, "
                f"fy={self._camera_matrix[1, 1]:.2f}, cx={self._camera_matrix[0, 2]:.2f}, "
                f"cy={self._camera_matrix[1, 2]:.2f}"
            )

    def _synchronized_callback(
        self,
        color_msg: Image,
        depth_msg: Image,
        detections_msg: AprilTagDetectionArray,
    ) -> None:
        if self._camera_matrix is None:
            return

        try:
            cv_color = self._cv_bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            cv_depth = self._cv_bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="16UC1"
            )
        except CvBridgeError as ex:
            self.get_logger().error(f"CvBridge 오류: {ex!s}")
            return

        img_h, img_w = cv_depth.shape

        for detection in detections_msg.detections:
            if detection.id != self._target_tag_id:
                continue

            if detection.decision_margin < self._min_decision_margin:
                self.get_logger().warn(
                    f"태그 {detection.id} 신뢰도 기각: {detection.decision_margin:.1f} < {self._min_decision_margin:.1f}",
                    throttle_duration_sec=2.0,
                )
                continue

            # 1. 2D 화소 중심 연산
            corners_x = [float(c.x) for c in detection.corners]
            corners_y = [float(c.y) for c in detection.corners]
            u_c = float(np.mean(corners_x))
            v_c = float(np.mean(corners_y))
            u_idx = round(u_c)
            v_idx = round(v_c)

            # 2. 5x5 ROI 깊이 중앙값 추출
            k = self._depth_roi_half_size
            u_min, u_max = max(0, u_idx - k), min(img_w, u_idx + k + 1)
            v_min, v_max = max(0, v_idx - k), min(img_h, v_idx + k + 1)

            depth_roi = cv_depth[v_min:v_max, u_min:u_max]
            valid_depths = depth_roi[depth_roi > 0]

            if valid_depths.size == 0:
                self.get_logger().warn(
                    f"태그 {detection.id} 유효 Depth 없음 (Min-Z 거리 제약 발생)",
                    throttle_duration_sec=2.0,
                )
                continue

            z_depth = float(np.median(valid_depths)) * 0.001

            # 3. 보정된 마커 규격(48mm) 기반 PnP 연산
            s = self._tag_size / 2.0
            obj_points = np.array(
                [
                    [-s, s, 0.0],
                    [s, s, 0.0],
                    [s, -s, 0.0],
                    [-s, -s, 0.0],
                ],
                dtype=np.float64,
            )
            img_points = np.array(
                [[float(c.x), float(c.y)] for c in detection.corners],
                dtype=np.float64,
            )

            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                img_points,
                self._camera_matrix,
                self._dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )

            if not success:
                success, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    img_points,
                    self._camera_matrix,
                    self._dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )

            if not success or tvec is None or rvec is None:
                continue

            z_pnp = float(tvec[2, 0])
            residual = abs(z_pnp - z_depth)

            # 잔차가 임계값을 초과할 경우 기각 처리
            if residual > self._residual_threshold_m:
                self.get_logger().warn(
                    f"잔차 초과 기각: |Z_pnp({z_pnp:.3f}) - Z_depth({z_depth:.3f})| = {residual * 1000:.1f}mm > {self._residual_threshold_m * 1000:.1f}mm",
                    throttle_duration_sec=1.0,
                )
                continue

            # 4. 카메라 Intrinsic 기반 3차원 역투영
            fx = self._camera_matrix[0, 0]
            fy = self._camera_matrix[1, 1]
            cx = self._camera_matrix[0, 2]
            cy = self._camera_matrix[1, 2]

            x_final = ((u_c - cx) * z_depth) / fx
            y_final = ((v_c - cy) * z_depth) / fy
            z_final = z_depth

            # 5. 회전 변환 쿼터니언 생성
            rot_mat, _ = cv2.Rodrigues(rvec)
            rot_scipy = R_scipy.from_matrix(rot_mat)
            q_final = rot_scipy.as_quat(canonical=False)

            # 6. TF2 변환 트리 브로드캐스트
            tf_msg = TransformStamped()
            tf_msg.header = depth_msg.header
            tf_msg.header.frame_id = self._target_frame
            tf_msg.child_frame_id = f"tag_{detection.id}_aligned"
            tf_msg.transform.translation.x = x_final
            tf_msg.transform.translation.y = y_final
            tf_msg.transform.translation.z = z_final
            tf_msg.transform.rotation.x = float(q_final[0])
            tf_msg.transform.rotation.y = float(q_final[1])
            tf_msg.transform.rotation.z = float(q_final[2])
            tf_msg.transform.rotation.w = float(q_final[3])
            self._tf_broadcaster.sendTransform(tf_msg)

            # 7. PoseStamped 토픽 발행
            pose_msg = PoseStamped()
            pose_msg.header = tf_msg.header
            pose_msg.pose.position.x = x_final
            pose_msg.pose.position.y = y_final
            pose_msg.pose.position.z = z_final
            pose_msg.pose.orientation = tf_msg.transform.rotation
            self._pub_target_pose.publish(pose_msg)

            # 8. RViz 3D 마커 발행 (노란색 사각형 플레이트 및 정밀 텍스트)
            self._publish_3d_markers(
                tf_msg.header,
                x_final,
                y_final,
                z_final,
                pose_msg.pose.orientation,
                detection.id,
            )

            # 9. 2D OpenCV AR 오버레이
            pts = np.array(
                [[round(c.x), round(c.y)] for c in detection.corners], np.int32
            ).reshape((-1, 1, 2))
            cv2.polylines(
                cv_color, [pts], isClosed=True, color=(0, 255, 0), thickness=2
            )
            cv2.circle(
                cv_color, (u_idx, v_idx), radius=4, color=(0, 0, 255), thickness=-1
            )

            tvec_fused = np.array([[x_final], [y_final], [z_final]], dtype=np.float64)
            cv2.drawFrameAxes(
                cv_color,
                self._camera_matrix,
                self._dist_coeffs,
                rvec,
                tvec_fused,
                self._tag_size * 0.8,
                thickness=2,
            )

            info_text = (
                f"ID:{detection.id} | Z:{z_final:.3f}m | Res:{residual * 1000:.1f}mm"
            )
            cv2.putText(
                cv_color,
                info_text,
                (u_idx - 60, max(20, v_idx - 35)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

            self._publish_count += 1
            if self._publish_count % 30 == 0:
                self.get_logger().info(
                    f"[추적 정밀 수렴] Tag {detection.id} -> "
                    f"X: {x_final:.3f}m, Y: {y_final:.3f}m, Z: {z_final:.3f}m | 잔차: {residual * 1000:.1f}mm"
                )

        # 10. 어노테이션 이미지 발행
        try:
            annotated_msg = self._cv_bridge.cv2_to_imgmsg(cv_color, encoding="bgr8")
            annotated_msg.header = color_msg.header
            self._pub_annotated_image.publish(annotated_msg)
        except CvBridgeError as ex:
            self.get_logger().error(f"이미지 브릿지 변환 실패: {ex!s}")

    def _publish_3d_markers(
        self,
        header,
        x: float,
        y: float,
        z: float,
        orientation,
        tag_id: int,
    ) -> None:
        # 마커 1: 노란색 사각 마커 플레이트
        cube_marker = Marker()
        cube_marker.header = header
        cube_marker.ns = "tag_bounding_box"
        cube_marker.id = tag_id
        cube_marker.type = Marker.CUBE
        cube_marker.action = Marker.ADD
        cube_marker.pose.position.x = x
        cube_marker.pose.position.y = y
        cube_marker.pose.position.z = z
        cube_marker.pose.orientation = orientation
        cube_marker.scale.x = self._tag_size
        cube_marker.scale.y = self._tag_size
        cube_marker.scale.z = 0.002  # 2mm 평면 플레이트
        # 노란색 적용 (Red 1.0, Green 1.0, Alpha 0.7)
        cube_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7)
        cube_marker.lifetime.sec = 0
        cube_marker.lifetime.nanosec = 250000000

        # 마커 2: 3D 텍스트 라벨 (자간 폭발 버그 차단 및 상단 밀착)
        text_marker = Marker()
        text_marker.header = header
        text_marker.ns = "tag_text"
        text_marker.id = tag_id + 1000
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = x
        text_marker.pose.position.y = y - (self._tag_size * 0.55)  # 상단 변에 밀착
        text_marker.pose.position.z = z
        text_marker.pose.orientation = orientation

        # 균등 3축 스케일 지정으로 RViz2 OGRE 자간 왜곡 방지
        font_size = 0.02  # 16mm 최적 폰트
        text_marker.scale.x = font_size
        text_marker.scale.y = font_size
        text_marker.scale.z = font_size
        text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        # 공백 제거 Compact 텍스트 포맷
        text_marker.text = f"Tag{tag_id}:Z={z:.3f}m"
        text_marker.lifetime.sec = 0
        text_marker.lifetime.nanosec = 250000000

        # Pylance 및 ROS 2 Sequence 호환 할당
        marker_array = MarkerArray()
        marker_array.markers = [cube_marker, text_marker]
        self._pub_markers.publish(marker_array)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RgbdTagFusionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
