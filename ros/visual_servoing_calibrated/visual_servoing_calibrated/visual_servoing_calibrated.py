import cv2
import math
import rclpy
import numpy as np

from time import sleep

from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg import Image, JointState


class Vp6242Controller(Node):
    """
    This class is used to control the vp6242 robot arm.

    Attributes:
        camera_rgb (Image): The RGB image from the camera.
        camera_depth (Image): The depth image from the camera.
        joint_states (JointState): The joint states of the robot arm.
        subscriber_camera_rgb (Subscription): The subscriber for the RGB camera.
        subscriber_camera_depth (Subscription): The subscriber for the depth camera.
        subscriber_joint_states (Subscription): The subscriber for the joint states.
        publisher (Publisher): The publisher for the joint commands.

    Methods:
        camera_rgb_callback: The callback function for the RGB camera.
        camera_depth_callback: The callback function for the depth camera.
        joint_states_callback: The callback function for the joint states.
        move_joint: Move the robot arm to a specific joint configuration.
    """

    def __init__(self):
        super().__init__('vp6242_controller')

        # IBVS gains
        self.lambda_i = 0.01
        self.lambda_j = 0.001

        # Focal length of the camera
        self.focal_length = 18.83623

        # Desired features
        self.desired_features = self.get_triangle_vertices(200, 512)

        # Define DH parameters for the VP6242 robot arm as a matrix
        # Each row corresponds to a joint: [Theta offset, d, a, alpha]
        self.dh_params = np.array([
            [0, 0.280, 0, -np.pi / 2],
            [-np.pi / 2, 0, 0.210, 0],
            [np.pi / 2, 0, -0.075, np.pi / 2],
            [0, 0.210, 0, -np.pi / 2],
            [0, 0, 0, np.pi / 2],
            [0, 0.070, 0, 0]
        ])

        # Initialize the camera images
        self.camera_rgb = None
        self.camera_depth = None

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Initialize the joint states
        self.joint_states = JointState()

        # Declare and get parameters
        self.declare_parameter('camera_image_topic', '/vp6242/camera/image')
        self.declare_parameter('camera_depth_topic', '/vp6242/camera/depth')
        self.declare_parameter('joint_states_topic', '/vp6242/joint_states')
        self.declare_parameter('joint_command_topic', '/vp6242/joint_command')

        camera_image_topic = self.get_parameter('camera_image_topic').get_parameter_value().string_value
        camera_depth_topic = self.get_parameter('camera_depth_topic').get_parameter_value().string_value
        joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        joint_command_topic = self.get_parameter('joint_command_topic').get_parameter_value().string_value

        # Initialize the camera subscribers
        self.subscriber_camera_rgb = self.create_subscription(
            Image,
            camera_image_topic,
            self.camera_rgb_callback,
            10
        )
        self.subscriber_camera_depth = self.create_subscription(
            Image,
            camera_depth_topic,
            self.camera_depth_callback,
            10
        )

        # Initialize the joint states subscriber
        self.subscriber_joint_states = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_states_callback,
            10
        )

        # Initialize the joint states publisher
        self.publisher = self.create_publisher(JointState, joint_command_topic, 10)

        # Timer control loop
        self.create_timer(0.5, self.timer_callback)

    def get_triangle_vertices(self, side_length, image_size):
        """
        Calculate the vertices of an equilateral triangle centered in an image.

        Parameters:
            side_length (float): The length of each side of the equilateral triangle.
            image_size (int): The size of the image (assumes a square image).

        Returns:
            vertices (np.array): The vertices of the equilateral triangle.
        """

        # Top vertex (A)
        top_x = image_size / 2
        top_y = image_size / 2 - (2 / 3) * (math.sqrt(3) / 2 * side_length)

        # Bottom-left vertex (B)
        bottom_left_x = image_size / 2 - side_length / 2
        bottom_left_y = image_size / 2 + (1 / 3) * (math.sqrt(3) / 2 * side_length)

        # Bottom-right vertex (C)
        bottom_right_x = image_size / 2 + side_length / 2
        bottom_right_y = image_size / 2 + (1 / 3) * (math.sqrt(3) / 2 * side_length)

        # Return the vertices
        vertices = np.array([top_x, top_y, bottom_left_x, bottom_left_y, bottom_right_x, bottom_right_y])

        return vertices

    def camera_rgb_callback(self, msg):
        """
        Callback function for the RGB camera.

        Args:
            msg (Image): The RGB image from the camera.
        """

        self.camera_rgb = msg

    def camera_depth_callback(self, msg):
        """
        Callback function for the depth camera.

        Args:
            msg (Image): The depth image from the camera.
        """

        self.camera_depth = msg

    def joint_states_callback(self, msg):
        """
        Callback function for the joint states.

        Args:
            msg (JointState): The joint states of the robot arm.
        """

        self.joint_states = msg

    def extract_features(self):
        """
        Extract features from the camera images.

        Returns:
            features (Optional[np.array]): The extracted features from the camera images.
            depth_features (Optional[np.array]): The extracted features from the depth image.
        """

        features = np.zeros(6)
        depth_features = np.zeros(3)

        # Check if the camera images are available
        if self.camera_rgb is None or self.camera_depth is None:
            self.get_logger().error('Camera images are not available.')
            return None, None

        try:
            # Convert ROS Image messages to OpenCV images
            rgb_image = self.bridge.imgmsg_to_cv2(self.camera_rgb, 'bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(self.camera_depth, 'mono8')

            # Convert RGB image to grayscale
            gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

            # Apply a threshold to detect the white area
            _, thresh = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)

            # Find contours in the thresholded image
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                self.get_logger().error('No contours found in the image.')
                return None, None

            # Find the largest contour (assuming it is the plate)
            largest_contour = max(contours, key=cv2.contourArea)

            # Get a bounding rectangle for the plate area
            x, y, w, h = cv2.boundingRect(largest_contour)
            roi = gray_image[y:y+h, x:x+w]

            # Apply Gaussian Blur and threshold to get a binary image
            gray_image_blurred = cv2.GaussianBlur(roi, (5, 5), 0)
            _, binary = cv2.threshold(gray_image_blurred, 200, 255, cv2.THRESH_BINARY_INV)

            # Find contours
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Filter contours by area and get the coordinates
            coordinates = [
                (int(cv2.moments(contour)["m10"] / cv2.moments(contour)["m00"]),
                 int(cv2.moments(contour)["m01"] / cv2.moments(contour)["m00"]))
                for contour in contours if cv2.contourArea(contour) > 25 and cv2.moments(contour)["m00"] != 0
            ]

            # If there less than 3 dots, raise an error
            if len(coordinates) < 3:
                self.get_logger().error('Less than 3 colored dots detected.')
                return None, None

            # If there are more than 3 dots, get the 3 closest to the center
            if len(coordinates) > 3:
                center = (w // 2, h // 2)
                coordinates.sort(key=lambda coord: np.linalg.norm(np.array(coord) - np.array(center)))
                coordinates = coordinates[:3]

            # Offset the coordinates based on the bounding rectangle
            coordinates = [(coord[0] + x, coord[1] + y) for coord in coordinates]

            # Separate the coordinates based on color
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
                self.get_logger().error('One or more colored dots are missing.')
                return None, None

            # Ensure the coordinates are in the order [redx, redy, greenx, greeny, bluex, bluey]
            ordered_coordinates = [red_dot[:2], green_dot[:2], blue_dot[:2]]

            # Assign the coordinates to features
            features = np.array(ordered_coordinates).flatten()

            # Extract depth features
            depth_features = np.array([
                depth_image[red_dot[1], red_dot[0]] / 255.0 * 5.0,
                depth_image[green_dot[1], green_dot[0]] / 255.0 * 5.0,
                depth_image[blue_dot[1], blue_dot[0]] / 255.0 * 5.0
            ])

        except cv2.error as e:
            self.get_logger().error(f'OpenCV error in extract_features: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in extract_features: {e}')
            raise e

        return features, depth_features

    def compute_jacobian_matrix(self, x, y, z):
        """
        Compute the Jacobian matrix for a single point.

        Args:
            x (float): The x-coordinate of the point.
            y (float): The y-coordinate of the point.
            z (float): The z-coordinate of the point.

        Returns:
            jacobian_matrix (np.array): The Jacobian matrix.
        """

        jacobian_matrix = np.array([
            [-self.focal_length / z, 0, x / z, x * y / self.focal_length, -
                (self.focal_length + x**2 / self.focal_length), y],
            [0, -self.focal_length / z, y / z, self.focal_length + y**2 / self.focal_length, -x * y / self.focal_length, -x]
        ])

        return jacobian_matrix

    def compute_image_jacobian(self, features, depth_features):
        """
        Compute the image Jacobian, based on P. Corke's Robotics, Vision and Control book.

        Args:
            features (np.array): The features from the camera images.
            depth_features (np.array): The depth features from the depth image.

        Returns:
            image_jacobian (np.array): The image Jacobian.
        """

        # Get the features
        red_x, red_y, green_x, green_y, blue_x, blue_y = features

        # Get the depth features
        red_z, green_z, blue_z = depth_features

        # Compute the image Jacobian
        red_matrix = self.compute_jacobian_matrix(red_x, red_y, red_z)
        green_matrix = self.compute_jacobian_matrix(green_x, green_y, green_z)
        blue_matrix = self.compute_jacobian_matrix(blue_x, blue_y, blue_z)

        # Stack the matrices
        image_jacobian = np.vstack((red_matrix, green_matrix, blue_matrix))

        return image_jacobian

    def compute_joint_velocities(self, camera_velocities):
        """
        Compute the joint velocities, using the VP6242 robot arm kinematics.

        Args:
            camera_velocities (np.array): The camera velocities.

        Returns:
            joint_velocities (np.array): The joint velocities.
        """

        joint_angles = self.joint_states.position

        T = np.eye(4)
        transformations = []

        for i, params in enumerate(self.dh_params):
            theta = joint_angles[i] + params[0]
            d = params[1]
            a = params[2]
            alpha = params[3]

            T_i = np.array([
                [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1]
            ])
            T = np.dot(T, T_i)

            transformations.append(T)

        T_end_effector = transformations[-1]
        p_end_effector = T_end_effector[:3, 3]

        J = np.zeros((6, 6))

        J[:3, 0] = np.cross(np.array([0, 0, 1]), p_end_effector)
        J[3:, 0] = np.array([0, 0, 1])

        for i in range(1, 6):
            T_i = transformations[i - 1]

            z_i = T_i[:3, 2]
            p_i = T_i[:3, 3]

            Jp = np.cross(z_i, p_end_effector - p_i)
            Jo = z_i

            J[:3, i] = Jp
            J[3:, i] = Jo

        jacobian_inverse = np.linalg.pinv(J)
        joint_velocities = np.dot(jacobian_inverse, camera_velocities)

        return joint_velocities

    def timer_callback(self):
        """
        Timer callback function.
        """

        # Feature extraction
        features, depth_features = self.extract_features()

        # Check if the features are available
        if features is None or depth_features is None:
            self.get_logger().error('Features are not available.')
            return

        # Compute error
        error = self.desired_features - features

        # Compute Image Jacobian
        image_jacobian = self.compute_image_jacobian(features, depth_features)
        inverse_jacobian = np.linalg.pinv(image_jacobian)

        # Image based visual servoing
        camera_velocities = self.lambda_i * np.dot(inverse_jacobian, error)

        # Compute joint velocities
        joint_velocities = self.lambda_j * self.compute_joint_velocities(camera_velocities)

        # Move the robot arm
        configuration = [
            {'name': 'Link1', 'velocity': joint_velocities[0]},
            {'name': 'Link2', 'velocity': joint_velocities[1]},
            {'name': 'Link3', 'velocity': joint_velocities[2]},
            {'name': 'Link4', 'velocity': joint_velocities[3]},
            {'name': 'Link5', 'velocity': joint_velocities[4]},
            {'name': 'EndEffector', 'velocity': joint_velocities[5]}
        ]

        self.move_joint(configuration)

    def move_joint(self, configuration):
        """
        Move the robot arm to a specific joint configuration.

        Args:
            configuration (list[object]): The joint configuration to move the robot arm to.
        """

        joint_command = JointState()

        joint_names = [joint['name'] for joint in configuration]
        joint_positions = [joint['position'] for joint in configuration if 'position' in joint]
        joint_velocities = [joint['velocity'] for joint in configuration if 'velocity' in joint]

        joint_command.name = joint_names
        joint_command.position = joint_positions
        joint_command.velocity = joint_velocities

        self.publisher.publish(joint_command)


def main(args=None):
    rclpy.init(args=args)

    vp6242_controller = Vp6242Controller()

    vp6242_controller.move_joint([
        {'name': 'Link1', 'position': 0.0},
        {'name': 'Link2', 'position': 0.0},
        {'name': 'Link3', 'position': np.pi / 2},
        {'name': 'Link4', 'position': 0.0},
        {'name': 'Link5', 'position': 0.0},
        {'name': 'EndEffector', 'position': 0.0}
    ])

    sleep(1)

    try:
        rclpy.spin(vp6242_controller)
    except KeyboardInterrupt:
        rclpy.logging.get_logger("Quitting").info('Done')

    vp6242_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
