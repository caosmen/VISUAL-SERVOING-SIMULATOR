import math
import numpy as np


def get_triangle_vertices(side_length, image_size):
    """"
    Get the vertices of an equilateral triangle.

    Args:
        side_length (float): The side length of the triangle.
        image_size (int): The size of the image.

    Returns:
        vertices (np.array): The vertices of the triangle.
    """

    top_x = image_size / 2
    top_y = image_size / 2 - (2 / 3) * (math.sqrt(3) / 2 * side_length)

    bottom_left_x = image_size / 2 - side_length / 2
    bottom_left_y = image_size / 2 + (1 / 3) * (math.sqrt(3) / 2 * side_length)

    bottom_right_x = image_size / 2 + side_length / 2
    bottom_right_y = image_size / 2 + (1 / 3) * (math.sqrt(3) / 2 * side_length)

    vertices = np.array([top_x, top_y, bottom_left_x, bottom_left_y, bottom_right_x, bottom_right_y])

    return vertices
