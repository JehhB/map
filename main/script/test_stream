#!/usr/bin/env python3

import argparse

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class MjpegStreamPublisher:
    def __init__(
        self,
        url,
        topic_name,
        rotation=0,
        flip_horizontal=False,
        flip_vertical=False,
        frame_id="camera_frame",
    ):
        """
        Initialize the MJPEG stream publisher.

        Args:
            url (str): URL of the MJPEG stream
            topic_name (str): ROS topic name to publish the images to
            rotation (int): Number of 90-degree clockwise rotations (0-3)
            flip_horizontal (bool): Whether to flip the image horizontally
            flip_vertical (bool): Whether to flip the image vertically
            frame_id (str): Frame ID to use in the ROS message header
        """
        self.url = url
        self.topic_name = topic_name
        self.rotation = rotation % 4  # Ensure rotation is between 0-3
        self.flip_horizontal = flip_horizontal
        self.flip_vertical = flip_vertical
        self.frame_id = frame_id

        # Initialize ROS node and publisher
        rospy.init_node("mjpeg_stream_publisher", anonymous=True)
        self.pub = rospy.Publisher(self.topic_name, Image, queue_size=10)
        self.bridge = CvBridge()

        # Rotation mapping (number of 90-degree rotations to OpenCV rotation code)
        self.rotation_map = {
            0: None,  # No rotation
            1: cv2.ROTATE_90_CLOCKWISE,
            2: cv2.ROTATE_180,
            3: cv2.ROTATE_90_COUNTERCLOCKWISE,
        }

        rospy.loginfo(f"MJPEG stream publisher initialized with URL: {self.url}")
        rospy.loginfo(f"Publishing to topic: {self.topic_name}")
        rospy.loginfo(
            f"Image processing: rotation={self.rotation*90}°, "
            f"flip_horizontal={self.flip_horizontal}, "
            f"flip_vertical={self.flip_vertical}"
        )

    def process_image(self, frame):
        """
        Process the image with the specified transformations.

        Args:
            frame: OpenCV image frame

        Returns:
            Processed OpenCV image frame
        """
        # Apply rotation if needed
        if self.rotation != 0:
            frame = cv2.rotate(frame, self.rotation_map[self.rotation])

        # Apply flipping if needed
        if self.flip_horizontal and self.flip_vertical:
            frame = cv2.flip(frame, -1)  # Flip both horizontally and vertically
        elif self.flip_horizontal:
            frame = cv2.flip(frame, 1)  # Flip horizontally
        elif self.flip_vertical:
            frame = cv2.flip(frame, 0)  # Flip vertically

        return frame

    def run(self):
        """
        Main loop to capture frames from MJPEG stream and publish to ROS topic.
        """
        # Open video stream
        cap = cv2.VideoCapture(self.url)

        if not cap.isOpened():
            rospy.logerr(f"Failed to open MJPEG stream at URL: {self.url}")
            return

        rospy.loginfo("Stream opened successfully, starting to publish frames...")

        rate = rospy.Rate(30)  # 30 Hz publishing rate

        while not rospy.is_shutdown():
            ret, frame = cap.read()

            if not ret:
                rospy.logwarn("Failed to receive frame, trying to reconnect...")
                cap.release()
                cap = cv2.VideoCapture(self.url)
                if not cap.isOpened():
                    rospy.logerr("Reconnection failed, stopping publisher")
                    break
                continue

            # Process the frame
            processed_frame = self.process_image(frame)

            # Convert to mono8 (grayscale)
            if len(processed_frame.shape) == 3:
                processed_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2GRAY)

            # Convert to ROS Image message and publish
            try:
                ros_image = self.bridge.cv2_to_imgmsg(processed_frame, "mono8")
                # Set current timestamp
                ros_image.header.stamp = rospy.Time.now()
                ros_image.header.frame_id = self.frame_id
                self.pub.publish(ros_image)
            except Exception as e:
                rospy.logerr(f"Error converting/publishing image: {e}")

            rate.sleep()

        # Clean up
        cap.release()
        rospy.loginfo("Stream publisher stopped")


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="MJPEG Stream Publisher for ROS")
    parser.add_argument(
        "--url", type=str, required=True, help="URL of the MJPEG stream"
    )
    parser.add_argument(
        "--topic",
        type=str,
        default="/camera/image_raw",
        help="ROS topic name to publish the images to (default: /camera/image_raw)",
    )
    parser.add_argument(
        "--rotation",
        type=int,
        default=0,
        choices=[0, 1, 2, 3],
        help="Number of 90-degree clockwise rotations (0-3, default: 0)",
    )
    parser.add_argument(
        "--flip-horizontal", action="store_true", help="Flip the image horizontally"
    )
    parser.add_argument(
        "--flip-vertical", action="store_true", help="Flip the image vertically"
    )
    parser.add_argument(
        "--frame-id",
        type=str,
        default="camera_frame",
        help="Frame ID for the ROS message header (default: camera_frame)",
    )

    args = parser.parse_args()

    try:
        publisher = MjpegStreamPublisher(
            url=args.url,
            topic_name=args.topic,
            rotation=args.rotation,
            flip_horizontal=args.flip_horizontal,
            flip_vertical=args.flip_vertical,
            frame_id=args.frame_id,
        )
        publisher.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
