import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from shigure.nodes.openpose.wrapper import OpenPoseWrapper

from openpose_ros2_msgs.msg import PoseKeyPointsList, PoseKeyPoint, PoseKeyPoints


class OpenPosePreviewNode(Node):

    openpose_wrapper: OpenPoseWrapper

    def __init__(self):
        super().__init__('openpose_node')

        self.openpose_wrapper = OpenPoseWrapper('/openpose')

        self._publisher = self.create_publisher(Image, '/openpose/preview', 10)
        self._pose_publisher = self.create_publisher(PoseKeyPointsList, '/openpose/pose_key_points', 10)

        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.get_color_callback, 10)

    def get_color_callback(self, image_raw: Image) -> None:
        try:
            bridge = CvBridge()
            image: np.ndarray = bridge.imgmsg_to_cv2(image_raw)

            result = self.openpose_wrapper.body_from_image(image)
            result_image = bridge.cv2_to_imgmsg(result.cvOutputData, 'rgb8')
            self._publisher.publish(result_image)

            # Convert to KeyPointsList
            pose_key_points_list_obj = PoseKeyPointsList()
            if isinstance(result.poseKeypoints, np.ndarray):
                pose_key_points_list = []
                for result_pose_key_points in result.poseKeypoints:
                    pose_key_points = []
                    for result_pose_key_point in result_pose_key_points:
                        x, y, score = result_pose_key_point
                        pose_key_points.append(PoseKeyPoint(x=x.item(), y=y.item(), score=score.item()))
                    pose_key_points_list.append(PoseKeyPoints(pose_key_points=pose_key_points))
                pose_key_points_list_obj = PoseKeyPointsList(pose_key_points_list=pose_key_points_list)
            self._pose_publisher.publish(pose_key_points_list_obj)

        except Exception as err:
            self.get_logger().error(err)


def main(args=None):
    rclpy.init(args=args)

    openpose_preview_node = OpenPosePreviewNode()

    try:
        rclpy.spin(openpose_preview_node)

    except KeyboardInterrupt:
        pass

    finally:
        # 終了処理
        openpose_preview_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
