import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage


class TagVisualizer(Node):
    def __init__(self):
        super().__init__("tag_visualizer")
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_detections = []
        self.tag_poses = {}  # { 'tag36h11:0': (tvec, rvec) }

        # 카메라 렌즈 정보 구독
        self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        # AprilTag 검출 결과 구독
        self.create_subscription(
            AprilTagDetectionArray, "/apriltag/detections", self.on_detections, 10
        )
        # TF 토픽 구독 (3D 위치 취득)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 10)

        # 컬러 영상 구독
        self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.on_image,
            qos_profile_sensor_data,
        )
        # 정렬된 뎁스 영상 구독 (컬러맵 변환용)
        self.create_subscription(
            Image,
            "/camera/aligned_depth_to_color/image_raw",
            self.on_depth_image,
            qos_profile_sensor_data,
        )

        # 마킹된 컬러 영상 및 뎁스 컬러맵 영상 토픽 발행
        self.image_pub = self.create_publisher(Image, "/camera/color/image_marked", 10)
        self.depth_colormap_pub = self.create_publisher(
            Image, "/camera/aligned_depth_to_color/image_colormap", 10
        )
        self.get_logger().info("Z거리 출력이 포함된 Tag Visualizer가 시작되었습니다.")

    def on_camera_info(self, msg):
        if self.camera_matrix is None and len(msg.k) == 9:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.dist_coeffs = (
                np.array(msg.d, dtype=np.float64)
                if len(msg.d) > 0
                else np.zeros(5, dtype=np.float64)
            )

    def on_detections(self, msg):
        self.latest_detections = msg.detections

    def on_tf(self, msg):
        for tf in msg.transforms:
            child = tf.child_frame_id
            if "tag" in child:
                trans = tf.transform.translation
                rot = tf.transform.rotation
                tvec = np.array([trans.x, trans.y, trans.z], dtype=np.float64)
                rvec = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]).as_rotvec()
                self.tag_poses[child] = (tvec, rvec)

    def on_image(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Image 변환 오류: {e}")
            return

        for detection in self.latest_detections:
            tag_id = detection.id
            cx, cy = int(detection.centre.x), int(detection.centre.y)

            # 1. 외곽선 사각형 그리기 (초록색)
            corners = np.array(
                [(int(pt.x), int(pt.y)) for pt in detection.corners],
                dtype=np.int32,
            )
            cv2.polylines(
                cv_image, [corners], isClosed=True, color=(0, 255, 0), thickness=2
            )

            # 2. 기본 ID 텍스트 표기 (노란색)
            cv2.putText(
                cv_image,
                f"Tag ID: {tag_id}",
                (cx - 40, cy - 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )

            # 3. 3차원 축 및 Z 거리 표시
            frame_candidates = [
                f"tag36h11:{tag_id}",
                f"36h11:{tag_id}",
                f"tag_{tag_id}",
            ]
            for frame in frame_candidates:
                if frame in self.tag_poses:
                    tvec, rvec = self.tag_poses[frame]
                    x_dist, y_dist, z_dist = tvec[0], tvec[1], tvec[2]
                    dist_3d = float(np.linalg.norm(tvec))

                    cv2.putText(
                        cv_image,
                        f"Z: {z_dist:.3f}m ({z_dist * 100:.1f}cm)",
                        (cx - 50, cy + 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 255, 0),
                        2,
                    )
                    cv2.putText(
                        cv_image,
                        f"Dist: {dist_3d:.3f}m",
                        (cx - 50, cy + 45),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (200, 200, 200),
                        1,
                    )

                    self.get_logger().info(
                        f"[Tag {tag_id}] X: {x_dist:.3f}m, Y: {y_dist:.3f}m, "
                        f"Z(수직거리): {z_dist:.3f}m, 3D직선거리: {dist_3d:.3f}m",
                        throttle_duration_sec=1.0,
                    )

                    if self.camera_matrix is not None:
                        cv2.drawFrameAxes(
                            cv_image,
                            self.camera_matrix,
                            self.dist_coeffs,
                            rvec,
                            tvec,
                            length=0.05,
                            thickness=3,
                        )
                    break

        marked_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        marked_msg.header = msg.header
        self.image_pub.publish(marked_msg)

    def on_depth_image(self, msg):
        # 구독자가 있을 때만 변환하여 CPU 및 대역폭 절약
        if self.depth_colormap_pub.get_subscription_count() == 0:
            return

        try:
            depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error(f"Depth Image 변환 오류: {e}")
            return

        max_dist_mm = 2000.0
        depth_clipped = np.clip(depth_mm, 0, max_dist_mm).astype(np.float32)
        depth_8u = 255 - (depth_clipped * (255.0 / max_dist_mm)).astype(np.uint8)

        colormap = cv2.applyColorMap(depth_8u, cv2.COLORMAP_JET)
        colormap[depth_mm == 0] = [0, 0, 0]

        out_msg = self.bridge.cv2_to_imgmsg(colormap, encoding="bgr8")
        out_msg.header = msg.header
        self.depth_colormap_pub.publish(out_msg)


def main():
    rclpy.init()
    node = TagVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
