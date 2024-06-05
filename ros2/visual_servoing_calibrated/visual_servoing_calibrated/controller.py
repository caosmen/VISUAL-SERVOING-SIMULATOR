import os
import numpy as np

from time import sleep

from rclpy.node import Node

from sensor_msgs.msg import JointState

from .state_machine import VP6242Machine
from .camera import VP6242CameraRGB, VP6242CameraDepth
from .joint import VP6242JointStates, VP6242JointController

from .utils import get_triangle_vertices


class VP6242Controller(Node):
    """
    Controller class for the VP6242 robot arm.

    Args:
        None

    Attributes:
        lambda_i (float): IBVS gain for the image.
        lambda_j (float): IBVS gain for the joints.
        focal_length (float): Focal length of the camera.
        sensor_size (tuple): Sensor size and resolution.
        camera_resolution (tuple): Camera resolution.
        sensor_pixel_size (float): Sensor pixel size.
        desired_features (np.ndarray): Desired features in the image.
        dh_params (np.ndarray): DH parameters of the robot arm.
        joint_limits (np.ndarray): Joints physical limits of the robot arm.
        camera_rgb (VP6242CameraRGB): VP6242CameraRGB object.
        camera_depth (VP6242CameraDepth): VP6242CameraDepth object.
        joint_states (VP6242JointStates): VP6242JointStates object.
        joint_controller (VP6242JointController): VP6242JointController object.

    Methods:
        init_subscribers_and_publishers: Initialize subscribers and publishers.
        main_loop: Timer callback function.
        move_joint: Move the robot arm to a specific joint configuration.
        generate_random_joint_values: Generate random joint values based on robot limits.
    """

    def __init__(self):
        super().__init__('vp6242_controller')

        # IBVS gains
        # Gain for the image
        self.lambda_i = 2.5 * 10 * (10 ** -2)
        # Gain for the joints
        self.lambda_j = 2.5 * 10 * (10 ** -2)

        # Camera parameters
        # Focal length of the camera
        self.focal_length = 12  # unit: mm
        # Sensor size and resolution
        self.sensor_size = (24, 24)  # unit: mm
        # Camera resolution
        self.camera_resolution = (512, 512)  # unit: pixels
        # Sensor pixel size
        self.sensor_pixel_size = np.mean(self.sensor_size) / np.mean(self.camera_resolution)  # unit: mm/pixel

        # Desired features in the image
        # self.desired_features = get_triangle_vertices(200, self.camera_resolution[0])
        self.desired_features = np.array([255, 230, 229, 281, 280, 281])

        # DH parameters of the robot arm
        # The DH parameters are in the format [offset, d, a, alpha]
        self.dh_params = np.array([
            [0, 0.280, 0, -np.pi / 2],
            [-np.pi / 2, 0, 0.210, 0],
            [np.pi / 2, 0, -0.075, np.pi / 2],
            [0, 0.210, 0, -np.pi / 2],
            [0, 0, 0, np.pi / 2],
            [-np.pi / 2, 0.070, 0, 0],
        ])

        # Joints physical limits of the robot arm
        # The joint limits are in the format [(low, high)] in degrees
        self.joint_limits = np.array([
            (-160, 160),
            (-120, 120),
            (19, 160),
            (-160, 160),
            (-120, 120),
            (-360, 360)
        ])

        # Initialize the camera objects
        self.camera_rgb = VP6242CameraRGB(self)
        self.camera_depth = VP6242CameraDepth(self)

        # Initialize the state machine object
        self.state_machine = VP6242Machine(self)

        # Initialize the joint states and joint controller objects
        self.joint_states = VP6242JointStates(self)
        self.joint_controller = VP6242JointController(self, self.dh_params)

        # Declare the parameters for the node
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_image_topic', '/vp6242/camera/image'),
                ('camera_depth_topic', '/vp6242/camera/depth'),
                ('joint_states_topic', '/vp6242/joint_states'),
                ('joint_command_topic', '/vp6242/joint_command'),
            ]
        )

        # Initialize the subscribers and publishers
        self.init_subscribers_and_publishers()

        # Create a timer to control the robot arm
        self.create_timer(0.1, self.main_loop)

    def init_subscribers_and_publishers(self):
        """
        Initialize the subscribers and publishers for the node.

        Args:
            None
        """

        camera_image_topic = self.get_parameter('camera_image_topic').get_parameter_value().string_value
        camera_depth_topic = self.get_parameter('camera_depth_topic').get_parameter_value().string_value
        joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        joint_command_topic = self.get_parameter('joint_command_topic').get_parameter_value().string_value

        self.camera_rgb.init_subscriber(camera_image_topic)
        self.camera_depth.init_subscriber(camera_depth_topic)
        self.joint_states.init_subscriber(joint_states_topic)

        self.publisher = self.create_publisher(JointState, joint_command_topic, 10)

    def main_loop(self):
        """
        Timer callback function to control the robot arm.

        Args:
            None
        """

        current_state = self.state_machine.current_state

        if current_state == self.state_machine.idle:
            features = self.camera_rgb.extract_features()

            if features is not None:
                self.state_machine.start_search()
            else:
                self.state_machine.start_track()
        elif current_state == self.state_machine.search:
            random_joint_values = self.generate_random_joint_values()
            configuration = self.joint_controller.create_joint_configuration(random_joint_values, type='position')

            self.move_joint(configuration)

            sleep(2)

            features = self.camera_rgb.extract_features()

            if features is not None:
                self.state_machine.found_target()

        elif current_state == self.state_machine.track:
            features = self.camera_rgb.extract_features()

            if features is not None:
                depth_features = self.camera_depth.extract_depth_features(features)

                error = self.desired_features - features

                image_jacobian = self.joint_controller.compute_image_jacobian(features, depth_features)

                inverse_jacobian = np.linalg.pinv(image_jacobian)

                camera_velocities = self.lambda_i * np.dot(inverse_jacobian, error.reshape(-1, 1))

                robot_jacobian, end_effector_transformation = self.joint_controller.compute_robot_jacobian(
                    self.joint_states.joint_states.position
                )
                end_effector_rotation = end_effector_transformation[:3, :3]

                inverse_robot_jacobian = robot_jacobian.T @ np.linalg.inv(
                    robot_jacobian @ robot_jacobian.T + 10 ** -5 * np.eye(6)
                )

                end_effector_rotation_kron = np.kron(np.eye(2), end_effector_rotation)

                camera_velocities_base = np.dot(end_effector_rotation_kron, camera_velocities)

                joint_velocities = (self.lambda_j * np.dot(inverse_robot_jacobian, camera_velocities_base)).flatten()
                configuration = self.joint_controller.create_joint_configuration(joint_velocities, type='velocity')

                self.move_joint(configuration)
            else:
                self.state_machine.lost_target()
        elif current_state == self.state_machine.end:
            pass

    def move_joint(self, configuration):
        """
        Move the robot arm to a specific joint configuration.

        Args:
            configuration (list[dict]): List of joint configurations.
        """

        joint_command = JointState()

        joint_names = [joint['name'] for joint in configuration]
        joint_positions = [joint['position'] for joint in configuration if 'position' in joint]
        joint_velocities = [joint['velocity'] for joint in configuration if 'velocity' in joint]

        joint_command.name = joint_names
        joint_command.position = joint_positions
        joint_command.velocity = joint_velocities

        self.publisher.publish(joint_command)

    def generate_random_joint_values(self):
        """
        Generate random joint values based on robot limits.

        Args:
            None
        """

        random_values_degrees = np.array([np.random.uniform(low, high) for (low, high) in self.joint_limits])
        random_values_radians = np.deg2rad(random_values_degrees)

        return random_values_radians
