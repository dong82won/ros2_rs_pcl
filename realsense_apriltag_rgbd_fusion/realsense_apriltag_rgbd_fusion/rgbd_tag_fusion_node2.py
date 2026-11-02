"""
ROS 2 Jazzy Jalisco - Lean RGB-D Tag Fusion ROS 2 Communication Wrapper.
Delegates heavy processing to RgbdTagFusionEngine and TagVisualizer.
"""

import message_filters
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, TransformStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray

from realsense_apriltag_rgbd_fusion.fusion_engine import RgbdTagFusionEngine
from realsense_apriltag_rgbd_fusion.types import CameraIntrinsics
from realsense_apriltag_rgbd_fusion.visualizer import TagVisualizer


class RgbdTagFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("rgbd_tag_fusion_node")

        # 1. Parameter Declarations
        self.declare_parameter("target_tag_id", 0)
        self.declare_parameter("tag_size", 0.048)
        self.declare_parameter("target_frame", "camera_color_optical_frame")
        self.declare_parameter("residual_threshold_m", 0.030)
        self.declare_parameter("min_decision_margin", 15.0)
        self.declare_parameter("depth_roi_half_size", 2)
        self.declare_parameter("sync_slop_sec", 0.060)

        self._target_tag_id: int = (
            self.get_parameter("target_tag_id").get_parameter_value().integer_value
        )
        self._target_frame: str = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self._min_decision_margin: float = (
            self.get_parameter("min_decision_margin").get_parameter_value().double_value
        )

        # 2. Engine and Visualizer Subsystems
        self._engine = RgbdTagFusionEngine(
            tag_size=self.get_parameter("tag_size").get_parameter_value().double_value,
            residual_threshold_m=self.get_parameter("residual_threshold_m")
            .get_parameter_value()
            .double_value,
            roi_half_size=self.get_parameter("depth_roi_half_size")
            .get_parameter_value()
            .integer_value,
        )
        self._visualizer = TagVisualizer()

        self.add_on_set_parameters_callback(self._on_parameter_change)

        # 3. Middleware Handlers & Interfaces
        self._intrinsics: CameraIntrinsics | None = None
        self._cv_bridge = CvBridge()
        self._tf_broadcaster = TransformBroadcaster(self)
        self._cb_group = ReentrantCallbackGroup()
        self._publish_count: int = 0

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # 4. Subscribers & Synchronizer
        self._sub_info = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self._info_cb,
            sensor_qos,
            callback_group=self._cb_group,
        )
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
        self._sub_det = message_filters.Subscriber(
            self,
            AprilTagDetectionArray,
            "/detections",
            qos_profile=sensor_qos,
            callback_group=self._cb_group,
        )

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_color, self._sub_depth, self._sub_det],
            queue_size=20,
            slop=self.get_parameter("sync_slop_sec").get_parameter_value().double_value,
        )
        self._sync.registerCallback(self._synchronized_callback)

        # 5. Publishers
        self._pub_pose = self.create_publisher(
            PoseStamped, "~/target_tag_pose", reliable_qos
        )
        self._pub_markers = self.create_publisher(
            MarkerArray, "~/target_tag_markers", reliable_qos
        )
        self._pub_ar_img = self.create_publisher(Image, "~/annotated_image", sensor_qos)

        self.get_logger().info(
            f"RgbdTagFusionNode 초기화 성공. Target ID: {self._target_tag_id}"
        )

    def _on_parameter_change(self, params: list[Parameter]) -> SetParametersResult:
        result = SetParametersResult(successful=True, reason="파라미터 갱신 성공")
        for p in params:
            if p.name == "target_tag_id" and p.value is not None:
                self._target_tag_id = int(p.value)
            elif p.name == "tag_size" and p.value is not None:
                self._engine.set_parameters(tag_size=float(p.value))
            elif p.name == "residual_threshold_m" and p.value is not None:
                self._engine.set_parameters(residual_threshold_m=float(p.value))
        return result

    def _info_cb(self, msg: CameraInfo) -> None:
        if self._intrinsics is None:
            # msg.k (Sequence/ndarray), msg.d (array.array) 타입이 from_params에 안전하게 매핑됨
            self._intrinsics = CameraIntrinsics.from_params(msg.k, msg.d)
            self.get_logger().info(
                f"카메라 Intrinsic 로드 완료: fx={self._intrinsics.fx:.2f}, "
                f"fy={self._intrinsics.fy:.2f}, cx={self._intrinsics.cx:.2f}, cy={self._intrinsics.cy:.2f}"
            )

    def _synchronized_callback(
        self, color_msg: Image, depth_msg: Image, det_msg: AprilTagDetectionArray
    ) -> None:
        # Pylance Type Narrowing 최적화 (로컬 변수 바인딩)
        intrinsics = self._intrinsics
        if intrinsics is None:
            return

        try:
            cv_color = self._cv_bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            cv_depth = self._cv_bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="16UC1"
            )
        except CvBridgeError as ex:
            self.get_logger().error(f"CvBridge 에러: {ex!s}")
            return

        for det in det_msg.detections:
            if (
                det.id != self._target_tag_id
                or det.decision_margin < self._min_decision_margin
            ):
                continue

            success, fused, reason = self._engine.process(
                det.id, det.corners, cv_depth, intrinsics
            )
            if not success or fused is None:
                self.get_logger().warn(
                    f"태그 {det.id} 기각: {reason}", throttle_duration_sec=2.0
                )
                continue

            # 1. TF 브로드캐스팅
            tf_msg = TransformStamped()
            tf_msg.header = depth_msg.header
            tf_msg.header.frame_id = self._target_frame
            tf_msg.child_frame_id = f"tag_{fused.tag_id}_aligned"
            tf_msg.transform.translation.x = fused.x
            tf_msg.transform.translation.y = fused.y
            tf_msg.transform.translation.z = fused.z_depth
            tf_msg.transform.rotation.x = fused.quaternion[0]
            tf_msg.transform.rotation.y = fused.quaternion[1]
            tf_msg.transform.rotation.z = fused.quaternion[2]
            tf_msg.transform.rotation.w = fused.quaternion[3]
            self._tf_broadcaster.sendTransform(tf_msg)

            # 2. PoseStamped 발행
            pose_msg = PoseStamped()
            pose_msg.header = tf_msg.header
            pose_msg.pose.position.x = fused.x
            pose_msg.pose.position.y = fused.y
            pose_msg.pose.position.z = fused.z_depth
            pose_msg.pose.orientation = tf_msg.transform.rotation
            self._pub_pose.publish(pose_msg)

            # 3. RViz 3D 마커 발행
            markers = self._visualizer.create_rviz_markers(
                tf_msg.header, fused, self._engine.tag_size
            )
            self._pub_markers.publish(markers)

            # 4. 2D OpenCV AR 오버레이
            self._visualizer.draw_ar_overlay(
                cv_color, fused, intrinsics, self._engine.tag_size
            )

            self._publish_count += 1
            if self._publish_count % 30 == 0:
                self.get_logger().info(
                    f"[모듈화 추적 중] Tag {fused.tag_id} -> Z: {fused.z_depth:.3f}m | Res: {fused.residual_m * 1000:.1f}mm"
                )

        # 5. 어노테이션 이미지 발행
        try:
            ar_msg = self._cv_bridge.cv2_to_imgmsg(cv_color, encoding="bgr8")
            ar_msg.header = color_msg.header
            self._pub_ar_img.publish(ar_msg)
        except CvBridgeError:
            pass


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
