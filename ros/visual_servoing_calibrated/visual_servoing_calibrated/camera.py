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

    def __init__(self, node, target_area_threshold=50, camera_threshold=120):
        super().__init__(node)

        self.camera_rgb = None

        self.target_area_threshold = target_area_threshold
        self.camera_threshold = camera_threshold

    def callback(self, msg):
        """
        The callback function for the camera subscriber.

        Args:
            msg (sensor_msgs.msg.Image): The image message.
        """

        self.camera_rgb = msg

    def extract_features(self):
        """
        Extract the features from the camera image.

        Returns:
            Optional[np.ndarray]: The extracted features.
        """

        features = np.zeros(6)

        if self.camera_rgb is None:
            return None

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(self.camera_rgb, 'bgr8')

            gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

            _, binary = cv2.threshold(gray_image, self.target_area_threshold, 255, cv2.THRESH_BINARY_INV)
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                return None

            largest_contour = max(contours, key=cv2.contourArea)

            x, y, w, h = cv2.boundingRect(largest_contour)
            roi = gray_image[y:y+h, x:x+w]

            gray_image_blurred = cv2.GaussianBlur(roi, (5, 5), 0)

            _, binary = cv2.threshold(gray_image_blurred, self.camera_threshold, 255, cv2.THRESH_BINARY_INV)
            contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                return None

            coordinates = [
                (int(cv2.moments(contour)["m10"] / cv2.moments(contour)["m00"]),
                 int(cv2.moments(contour)["m01"] / cv2.moments(contour)["m00"]))
                for contour in contours if cv2.contourArea(contour) > 25 and cv2.moments(contour)["m00"] != 0
            ]

            if len(coordinates) < 3:
                return None

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
                return None

            ordered_coordinates = [red_dot[:2], green_dot[:2], blue_dot[:2]]
            features = np.array(ordered_coordinates).flatten()

        except cv2.error as e:
            self.node.get_logger().error(f'OpenCV error in extract_features: {e}')

            return None

        except Exception as e:
            self.node.get_logger().error(f'Error in extract_features: {e}')

            return None

        return features


class CameraDepth(CameraBase):
    """
    Class for the depth camera.

    Args:
        node (rclpy.node.Node): The ROS node.

    Attributes:
        camera_depth (sensor_msgs.msg.Image): The depth camera image.

    Methods:
        callback: The callback function for the camera subscriber.
        extract_depth_features: Extract the depth features from the camera image.
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

    def extract_depth_features(self, features):
        """
        Extract the depth features from the camera image.

        Args:
            features (np.ndarray): The RGB features.

        Returns:
            Optional[np.ndarray]: The extracted depth features.
        """

        if features is None:
            return None

        if self.camera_depth is None:
            return None

        red_dot = features[:2]
        green_dot = features[2:4]
        blue_dot = features[4:]

        depth_image = self.bridge.imgmsg_to_cv2(self.camera_depth, 'mono8')

        depth_features = np.array([
            depth_image[red_dot[1], red_dot[0]] / 255.0 * 5.0,
            depth_image[green_dot[1], green_dot[0]] / 255.0 * 5.0,
            depth_image[blue_dot[1], blue_dot[0]] / 255.0 * 5.0
        ])

        return depth_features
