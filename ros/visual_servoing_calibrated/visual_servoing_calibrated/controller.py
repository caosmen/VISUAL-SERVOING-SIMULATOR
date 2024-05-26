import numpy as np

from rclpy.node import Node

from sensor_msgs.msg import JointState

from .camera import CameraRGB, CameraDepth
from .joint import JointStates, JointController

from .utils import get_triangle_vertices


class Vp6242Controller(Node):
    """
    Controller class for the VP6242 robot arm.

    Args:
        None

    Attributes:
        lambda_i (float): IBVS gain for the image.
        lambda_j (float): IBVS gain for the joints.
        focal_length (float): Focal length of the camera.
        desired_features (np.ndarray): Desired features in the image.
        dh_params (np.ndarray): DH parameters of the robot arm.
        camera_rgb (CameraRGB): RGB camera object.
        camera_depth (CameraDepth): Depth camera object.
        joint_states (JointStates): Joint states object.
        joint_controller (JointController): Joint controller object.
        publisher (rclpy.publisher.Publisher): Publisher for the joint commands.

    Methods:
        init_subscribers_and_publishers: Initialize subscribers and publishers.
        timer_callback: Timer callback function.
        move_joint: Move the robot arm to a specific joint configuration.
    """

    def __init__(self):
        super().__init__('vp6242_controller')

        # IBVS gains
        # Gain for the image
        self.lambda_i = 10 * (10 ** -2)
        # Gain for the joints
        self.lambda_j = 10 * (10 ** -2)

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
        self.desired_features = np.array([255, 282, 238, 315, 271, 315])

        # DH parameters of the robot arm
        # The DH parameters are in the format [offset, d, a, alpha]
        self.dh_params = np.array([
            [0, 0.280, 0, -np.pi / 2],
            [-np.pi / 2, 0, 0.210, 0],
            [np.pi / 2, 0, -0.075, np.pi / 2],
            [0, 0.210, 0, -np.pi / 2],
            [0, 0, 0, np.pi / 2],
            [0, 0.070, 0, 0]
        ])

        # Initialize the camera objects
        self.camera_rgb = CameraRGB(self)
        self.camera_depth = CameraDepth(self)

        # Initialize the joint states and joint controller objects
        self.joint_states = JointStates(self)
        self.joint_controller = JointController(self, self.dh_params)

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
        self.create_timer(0.5, self.timer_callback)

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

    def timer_callback(self):
        """
        Timer callback function to control the robot arm.

        Args:
            None
        """

        # Extract the features from the camera images
        features, depth_features = self.camera_rgb.extract_features(self.camera_depth)

        # If the features are not available, return
        if features is None or depth_features is None:
            return

        # Compute the error between the desired features and the current features
        error = self.desired_features - features

        # Compute the image Jacobian and its pseudo-inverse
        image_jacobian = self.joint_controller.compute_image_jacobian(features, depth_features)

        # Compute the inverse of the image Jacobian
        inverse_jacobian = image_jacobian.T @ np.linalg.inv(
            image_jacobian @ image_jacobian.T + 0.01**2 * np.eye(6)
        )

        # Compute the camera velocities
        camera_velocities = self.lambda_i * np.dot(inverse_jacobian, error.reshape(-1, 1))

        # Compute the robot Jacobian and camera rotation matrix
        robot_jacobian, camera_rotation = self.joint_controller.compute_robot_jacobian(
            self.joint_states.joint_states.position[1:7]
        )

        # Compute the inverse of the robot Jacobian
        inverse_robot_jacobian = robot_jacobian.T @ np.linalg.inv(
            robot_jacobian @ robot_jacobian.T + 0.01**2 * np.eye(6)
        )

        # Since the rotation matrix in orthogonal, the inverse is the transpose
        inverse_camera_rotation = camera_rotation.T
        # Apply the nkron operator to the inverse camera rotation matrix
        inverse_camera_rotation_kron = np.kron(np.eye(2), inverse_camera_rotation)

        # Compute the camera velocities vector in the base frame of the robot
        camera_velocities_base = np.dot(inverse_camera_rotation_kron, camera_velocities)

        # Compute the joint velocities
        joint_velocities = self.lambda_j * np.dot(inverse_robot_jacobian, camera_velocities_base)

        # Compute the joint configuration
        configuration = self.joint_controller.create_joint_configuration(joint_velocities.flatten())

        # Move the robot arm to the new joint configuration
        self.move_joint(configuration)

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
