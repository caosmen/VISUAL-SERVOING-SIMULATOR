import rclpy
import numpy as np

from time import sleep

from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState


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

        # Initialize the camera images
        self.camera_rgb = Image()
        self.camera_depth = Image()

        # Initialize the joint states
        self.joint_states = JointState()

        # Initialize the camera subscribers
        self.subscriber_camera_rgb = self.create_subscription(
            Image,
            '/vp6242/camera/image',
            self.camera_rgb_callback,
            10
        )
        self.subscriber_camera_depth = self.create_subscription(
            Image,
            '/vp6242/camera/depth',
            self.camera_depth_callback,
            10
        )

        # Initialize the joint states subscriber
        self.subscriber_joint_states = self.create_subscription(
            JointState,
            '/vp6242/joint_states',
            self.joint_states_callback,
            10
        )

        # Initialize the joint states publisher
        self.publisher = self.create_publisher(JointState, '/vp6242/joint_command', 10)

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

    def move_joint(self, configuration):
        """
        Move the robot arm to a specific joint configuration.

        Args:
            configuration (list[tuple]): The joint configuration to move the robot arm to.
        """

        joint_command = JointState()

        joint_command.name = [joint[0] for joint in configuration]
        joint_command.position = [joint[1] for joint in configuration]
        joint_command.velocity = [joint[2] for joint in configuration]

        self.publisher.publish(joint_command)


def main(args=None):
    rclpy.init(args=args)

    vp6242_controller = Vp6242Controller()


if __name__ == '__main__':
    main()
