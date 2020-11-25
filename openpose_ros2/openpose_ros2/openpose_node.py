import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from shigure.nodes.openpose.wrapper import OpenPoseWrapper


class OpenPosePreviewNode(Node):

    openpose_wrapper: OpenPoseWrapper

    def __init__(self):
        super().__init__('openpose_node')

        self.openpose_wrapper = OpenPoseWrapper('/openpose')

        self._publisher = self.create_publisher(Image, '/openpose/preview', 10)

        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.get_color_callback, 10)

    def get_color_callback(self, image_raw: Image) -> None:
        try:
            bridge = CvBridge()
            image: np.ndarray = bridge.imgmsg_to_cv2(image_raw)

            result = self.openpose_wrapper.body_from_image(image)
            result_image = bridge.cv2_to_imgmsg(result.cvOutputData, 'rgb8')
            self._publisher.publish(result_image)
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
