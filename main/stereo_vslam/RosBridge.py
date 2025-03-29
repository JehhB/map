import numpy as np
from cv_bridge import CvBridge
from PIL.Image import Image
from rospy import Publisher, Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as ImageRos


class RosBridge:
    bridge: CvBridge

    left_image_pub: Publisher
    right_image_pub: Publisher
    left_camera_info_pub: Publisher
    right_camera_info_pub: Publisher

    def __init__(self):
        self.bridge = CvBridge()

        self.left_image_pub = Publisher(
            "/stereo_camera/left/image_raw", ImageRos, queue_size=10
        )
        self.right_image_pub = Publisher(
            "/stereo_camera/right/image_raw", ImageRos, queue_size=10
        )
        self.left_camera_info_pub = Publisher(
            "/stereo_camera/left/camera_info", CameraInfo, queue_size=10
        )
        self.right_camera_info_pub = Publisher(
            "/stereo_camera/right/camera_info", CameraInfo, queue_size=10
        )

    def _image_to_imgmsg(self, img: Image) -> ImageRos:
        img_np = np.array(img)
        return self.bridge.cv2_to_imgmsg(img_np, encoding="rgb8")

    def send_stereo_camera_info(
        self, left_camera_info: CameraInfo, right_camera_info: CameraInfo
    ):
        self.left_camera_info_pub.publish(left_camera_info)
        self.right_camera_info_pub.publish(right_camera_info)

    def send_stereo_image(self, left_image: Image, right_image: Image):
        current_time = Time.now()

        left_image_msg = self._image_to_imgmsg(left_image)
        right_image_msg = self._image_to_imgmsg(right_image)

        left_image_msg.header.stamp = current_time
        right_image_msg.header.stamp = current_time

        self.left_image_pub.publish(left_image_msg)
        self.right_image_pub.publish(right_image_msg)

    def destroy(self):
        self.left_image_pub.unregister()
        self.right_image_pub.unregister()
        self.left_camera_info_pub.unregister()
        self.right_camera_info_pub.unregister()
