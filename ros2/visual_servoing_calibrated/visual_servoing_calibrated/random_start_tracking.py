import rclpy
import numpy as np

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

    rclpy.spin(vp6242_controller)

    vp6242_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
