import rclpy
import numpy as np

from time import sleep

from .controller import VP6242Controller

np.set_printoptions(precision=3, suppress=True)


def main(args=None):
    """
    Main function to initialize and run the ROS node.

    Args:
        args (list, optional): Command-line arguments.
    """

    rclpy.init(args=args)

    vp6242_controller = VP6242Controller()

    start_position = np.array([0.0, 0.0, np.pi / 2, 0.0, 0.0, 0.0])
    configuration = vp6242_controller.joint_controller.create_joint_configuration(start_position)

    vp6242_controller.move_joint(configuration)

    sleep(2)

    rclpy.spin(vp6242_controller)

    vp6242_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
