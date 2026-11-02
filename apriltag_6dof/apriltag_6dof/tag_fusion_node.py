"""카메라 중심 다중 AprilTag 6-DoF 추정 및 AR 오버레이 발행 ROS 2 노드."""

from typing import cast

import message_filters
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from apriltag_6dof.models import CameraIntrinsics, Pose6DoF, TagCorner2D
from apriltag_6dof.pnp_solver import solve_tag_pnp
from apriltag_6dof.visualizer import render_ar_overlay


class TagFusionNode(Node):
    """카메라 광학 프레임을 기준으로 시야 내 다중 마커의 6-DoF 상태를 일괄 브로드캐스트하는 노드."""

    def __init__(self) -> None:
        super().__init__("tag_fusion_node")

        # 1. 런타임 동적 파라미터 선언 및 로드
        self.declare_parameter("track_all_tags", True)
        self.declare_parameter("target_tag_id", 0)
        self.declare_parameter("tag_size", 0.048)
        self.declare_parameter("target_frame", "camera_color_optical_frame")
        self.declare_parameter("min_decision_margin", 15.0)
        self.declare_parameter("sync_slop_sec", 0.050)
        self.declare_parameter("reprojection_error_thresh_px", 3.0)
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")

        self._track_all_tags: bool = (
            self.get_parameter("track_all_tags").get_parameter_value().bool_value
        )
        self._target_tag_id: int = (
            self.get_parameter("target_tag_id").get_parameter_value().integer_value
        )
        self._tag_size: float = (
            self.get_parameter("tag_size").get_parameter_value().double_value
        )
        self._target_frame: str = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self._min_margin: float = (
            self.get_parameter("min_decision_margin").get_parameter_value().double_value
        )
        self._sync_slop: float = (
            self.get_parameter("sync_slop_sec").get_parameter_value().double_value
        )
        self._reproj_thresh: float = (
            self.get_parameter("reprojection_error_thresh_px")
            .get_parameter_value()
            .double_value
        )
        image_topic: str = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        camera_info_topic: str = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )

        # 2. 내부 상태 및 어댑터 객체 초기화
        self._intrinsics: CameraIntrinsics | None = None
        self._cv_bridge = CvBridge()
        self._tf_broadcaster = TransformBroadcaster(self)
        self._cb_group = ReentrantCallbackGroup()
        self._frame_count: int = 0

        # 3. 통신 품질(QoS) 정의
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

        # 4. 카메라 내부 파라미터 구독 (1회 수신 후 해제 전담)
        self._sub_camera_info = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._on_camera_info,
            sensor_qos,
            callback_group=self._cb_group,
        )

        # 5. 영상 스트림과 태그 검출 토픽 간 근사 시간 동기화
        self._sub_color = message_filters.Subscriber(
            self,
            Image,
            image_topic,
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

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_color, self._sub_detections],
            queue_size=10,
            slop=self._sync_slop,
        )
        self._sync.registerCallback(self._on_synchronized_frame)

        # 6. 최종 6-DoF 결과 및 시각화 퍼블리셔 등록
        self._pub_pose = self.create_publisher(
            PoseStamped, "~/target_tag_pose", reliable_qos
        )
        self._pub_markers = self.create_publisher(
            MarkerArray, "~/target_tag_markers", reliable_qos
        )
        self._pub_annotated_image = self.create_publisher(
            Image, "~/annotated_image", reliable_qos
        )

        self.get_logger().info(
            f"TagFusionNode 준비 완료 | 전체 추적 모드: {self._track_all_tags}, 마커 크기: {self._tag_size * 1000:.1f}mm"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        """카메라 고유 내부 파라미터를 1회 캐싱한 뒤 구독을 즉시 파기합니다."""
        if self._intrinsics is not None:
            return

        camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        dist_coeffs = (
            np.array(msg.d[:5], dtype=np.float64).reshape((5, 1))
            if len(msg.d) >= 5
            else np.zeros((5, 1), dtype=np.float64)
        )
        self._intrinsics = CameraIntrinsics(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
        )
        self.get_logger().info(
            f"카메라 Intrinsic 로드 완료: fx={camera_matrix[0, 0]:.2f}, fy={camera_matrix[1, 1]:.2f}"
        )
        self.destroy_subscription(self._sub_camera_info)

    def _on_synchronized_frame(
        self,
        color_msg: Image,
        detections_msg: AprilTagDetectionArray,
    ) -> None:
        """동기화된 영상과 다중 검출 결과를 결합하여 일괄 처리합니다."""
        if self._intrinsics is None:
            return

        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except CvBridgeError as ex:
            self.get_logger().error(f"CvBridge 디코딩 실패: {ex!s}")
            return

        annotated_image = cv_image
        transforms_to_broadcast: list[TransformStamped] = []
        valid_poses: list[Pose6DoF] = []

        # 1. 시야 내 모든 태그 순회 및 연산 수집
        for det in detections_msg.detections:
            if not self._track_all_tags and det.id != self._target_tag_id:
                continue

            if det.decision_margin < self._min_margin:
                continue

            tag_data = TagCorner2D(
                tag_id=det.id,
                corners=tuple((float(c.x), float(c.y)) for c in det.corners),
                center=(float(det.centre.x), float(det.centre.y)),
                decision_margin=float(det.decision_margin),
            )

            pose = solve_tag_pnp(tag_data, self._tag_size, self._intrinsics)
            if pose is None:
                continue

            if pose.reprojection_error > self._reproj_thresh:
                continue

            valid_poses.append(pose)

            # TF 메시지 배열 구성
            tf_msg = TransformStamped()
            tf_msg.header = color_msg.header
            tf_msg.header.frame_id = self._target_frame
            tf_msg.child_frame_id = f"tag_{pose.tag_id}"
            tf_msg.transform.translation.x = pose.translation[0]
            tf_msg.transform.translation.y = pose.translation[1]
            tf_msg.transform.translation.z = pose.translation[2]
            tf_msg.transform.rotation.x = pose.quaternion[0]
            tf_msg.transform.rotation.y = pose.quaternion[1]
            tf_msg.transform.rotation.z = pose.quaternion[2]
            tf_msg.transform.rotation.w = pose.quaternion[3]
            transforms_to_broadcast.append(tf_msg)

            # AR 오버레이 누적 렌더링
            annotated_image = render_ar_overlay(
                annotated_image,
                tag_data,
                pose,
                self._intrinsics,
                self._tag_size,
            )

        # 2. 수집된 다중 TF 일괄 브로드캐스트
        if transforms_to_broadcast:
            self._tf_broadcaster.sendTransform(transforms_to_broadcast)

        # 3. 3D 마커 일괄 발행 및 대표 포즈 발행
        if valid_poses:
            self._publish_3d_markers(valid_poses, color_msg.header)
            # PoseStamped는 가장 먼저 검출된(또는 대표적인) 마커 기준 1개만 발행
            self._publish_pose(valid_poses[0], color_msg.header)

            self._frame_count += 1
            if self._frame_count % 30 == 0:
                p = valid_poses[0]
                self.get_logger().info(
                    f"검출 태그: {[v.tag_id for v in valid_poses]} | Tag_{p.tag_id} Z:{p.translation[2]:.3f}m | Err:{p.reprojection_error:.2f}px"
                )

        # 4. 최종 AR 영상 발행
        try:
            ar_msg = cast(
                Image,
                self._cv_bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8"),
            )
            ar_msg.header = color_msg.header
            self._pub_annotated_image.publish(ar_msg)
        except CvBridgeError as ex:
            self.get_logger().error(f"CvBridge 인코딩 실패: {ex!s}")

    def _publish_pose(self, pose: Pose6DoF, header) -> None:
        """표준 PoseStamped 메시지를 발행합니다."""
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = self._target_frame
        pose_msg.pose.position.x = pose.translation[0]
        pose_msg.pose.position.y = pose.translation[1]
        pose_msg.pose.position.z = pose.translation[2]
        pose_msg.pose.orientation.x = pose.quaternion[0]
        pose_msg.pose.orientation.y = pose.quaternion[1]
        pose_msg.pose.orientation.z = pose.quaternion[2]
        pose_msg.pose.orientation.w = pose.quaternion[3]
        self._pub_pose.publish(pose_msg)

    def _publish_3d_markers(self, poses: list[Pose6DoF], header) -> None:
        """다수의 마커 플레이트를 하나의 MarkerArray에 담아 일괄 발행합니다."""
        marker_array = MarkerArray()
        # Pylance Sequence[Unknown] 오류 우회를 위한 표준 리스트 객체 생성
        markers_list = []

        for pose in poses:
            plate = Marker()
            plate.header = header
            plate.header.frame_id = self._target_frame
            plate.ns = "tag_plate"
            plate.id = pose.tag_id
            plate.type = Marker.CUBE
            plate.action = Marker.ADD
            plate.pose.position.x = pose.translation[0]
            plate.pose.position.y = pose.translation[1]
            plate.pose.position.z = pose.translation[2]
            plate.pose.orientation.x = pose.quaternion[0]
            plate.pose.orientation.y = pose.quaternion[1]
            plate.pose.orientation.z = pose.quaternion[2]
            plate.pose.orientation.w = pose.quaternion[3]
            plate.scale.x = self._tag_size
            plate.scale.y = self._tag_size
            plate.scale.z = 0.002
            plate.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
            plate.lifetime.nanosec = 250000000

            text = Marker()
            text.header = header
            text.header.frame_id = self._target_frame
            text.ns = "tag_text"
            text.id = pose.tag_id + 1000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pose.translation[0]
            text.pose.position.y = pose.translation[1] - (self._tag_size * 0.6)
            text.pose.position.z = pose.translation[2]
            text.scale.x = 0.02
            text.scale.y = 0.02
            text.scale.z = 0.02
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f"Tag{pose.tag_id}:Z={pose.translation[2]:.3f}m"
            text.lifetime.nanosec = 250000000

            # 파이썬 표준 리스트에 먼저 객체들을 누적
            markers_list.extend([plate, text])

        # 루프 종료 후 ROS 2 메시지 객체에 리스트를 직접 할당
        marker_array.markers = markers_list
        self._pub_markers.publish(marker_array)


def main(args: list[str] | None = None) -> None:
    """노드 초기화 및 멀티스레드 실행 진입점."""
    rclpy.init(args=args)
    node = TagFusionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
