import rclpy
import numpy as np

from time import sleep

from .controller import Vp6242Controller

np.set_printoptions(precision=3, suppress=True)


def main(args=None):
    """
    Main function to initialize and run the ROS node.

    Args:
        args (list, optional): Command-line arguments.
    """

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
