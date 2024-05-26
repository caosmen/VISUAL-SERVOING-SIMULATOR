import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CameraBase:
    """
    Base class for camera objects.

    Args:
        node (rclpy.node.Node): The ROS node.

    Methods:
        init_subscriber: Initialize the subscriber for the camera topic.
        callback: The callback function for the camera subscriber.
    """

    def __init__(self, node):
        self.node = node
        self.bridge = CvBridge()

    def init_subscriber(self, topic):
        """
        Initialize the subscriber for the camera topic.

        Args:
            topic (str): The topic name.
        """

        self.node.create_subscription(Image, topic, self.callback, 10)

    def callback(self, msg):
        """
        The callback function for the camera subscriber.

        Args:
            msg (sensor_msgs.msg.Image): The image message.
        """

        raise NotImplementedError("This method should be implemented by subclasses")


class CameraRGB(CameraBase):
    """
    Class for the RGB camera.

    Args:
        node (rclpy.node.Node): The ROS node.

    Attributes:
        camera_rgb (sensor_msgs.msg.Image): The RGB camera image.

    Methods:
        callback: The callback function for the camera subscriber.
        extract_features: Extract the features from the camera image.
    """

    def __init__(self, node):
        super().__init__(node)

        self.camera_rgb = None

    def callback(self, msg):
        """
        The callback function for the camera subscriber.

        Args:
            msg (sensor_msgs.msg.Image): The image message.
        """

        self.camera_rgb = msg

    def extract_features(self, camera_depth):
        """
        Extract the features from the camera image.

        Args:
            camera_depth (CameraDepth): The depth camera object.

        Returns:
            Optional[np.ndarray]: The extracted features.
            Optional[np.ndarray]: The extracted depth features.
        """

        features = np.zeros(6)
        depth_features = np.zeros(3)

        if self.camera_rgb is None or camera_depth.camera_depth is None:
            return None, None

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(self.camera_rgb, 'bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(camera_depth.camera_depth, 'mono8')

            gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                return None, None

            largest_contour = max(contours, key=cv2.contourArea)

            x, y, w, h = cv2.boundingRect(largest_contour)
            roi = gray_image[y:y+h, x:x+w]

            gray_image_blurred = cv2.GaussianBlur(roi, (5, 5), 0)

            _, binary = cv2.threshold(gray_image_blurred, 200, 255, cv2.THRESH_BINARY_INV)
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            coordinates = [
                (int(cv2.moments(contour)["m10"] / cv2.moments(contour)["m00"]),
                 int(cv2.moments(contour)["m01"] / cv2.moments(contour)["m00"]))
                for contour in contours if cv2.contourArea(contour) > 25 and cv2.moments(contour)["m00"] != 0
            ]

            if len(coordinates) < 3:
                return None, None

            if len(coordinates) > 3:
                center = (w // 2, h // 2)
                coordinates.sort(key=lambda coord: np.linalg.norm(np.array(coord) - np.array(center)))
                coordinates = coordinates[:3]

            coordinates = [(coord[0] + x, coord[1] + y) for coord in coordinates]

            red_dot = None
            green_dot = None
            blue_dot = None

            for coord in coordinates:
                b, g, r = rgb_image[coord[1], coord[0]]

                if r > g and r > b:
                    red_dot = coord
                elif g > r and g > b:
                    green_dot = coord
                elif b > r and b > g:
                    blue_dot = coord

            if not (red_dot and green_dot and blue_dot):
                return None, None

            ordered_coordinates = [red_dot[:2], green_dot[:2], blue_dot[:2]]
            features = np.array(ordered_coordinates).flatten()

            depth_features = np.array([
                depth_image[red_dot[1], red_dot[0]] / 255.0 * 5.0,
                depth_image[green_dot[1], green_dot[0]] / 255.0 * 5.0,
                depth_image[blue_dot[1], blue_dot[0]] / 255.0 * 5.0
            ])

        except cv2.error as e:
            self.node.get_logger().error(f'OpenCV error in extract_features: {e}')

            return None, None

        except Exception as e:
            self.node.get_logger().error(f'Error in extract_features: {e}')

            return None, None

        return features, depth_features


class CameraDepth(CameraBase):
    """
    Class for the depth camera.

    Args:
        node (rclpy.node.Node): The ROS node.

    Attributes:
        camera_depth (sensor_msgs.msg.Image): The depth camera image.

    Methods:
        callback: The callback function for the camera subscriber.
    """

    def __init__(self, node):
        super().__init__(node)

        self.camera_depth = None

    def callback(self, msg):
        """
        The callback function for the camera subscriber.

        Args:
            msg (sensor_msgs.msg.Image): The image message.
        """

        self.camera_depth = msg
