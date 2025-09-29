import time
import numpy as np
from mbot_bridge.api import MBot


def find_min_dist(ranges, thetas):
    """Finds the length and angle of the minimum ray in the scan.

    Make sure you ignore any rays with length 0! Those are invalid.

    Args:
        ranges (list): The length of each ray in the Lidar scan.
        thetas (list): The angle of each ray in the Lidar scan.

    Returns:
        tuple: The length and angle of the shortest ray in the Lidar scan.
    """
    min_dist, min_angle = None, None

    # TODO: Find the length and angle of the shortest distance in the ray.
    
    # Complete the provided function, find_min_dist(), in wall_follower.py 
    # so that it finds the length and angle of the shortest ray. For example, 
    # if the smallest range in ranges is the tenth element at index 9, 
    # the function should return the tuple ranges[9], thetas[9]

    min_value = float('inf')
    min_index = len(ranges)
    for index, value in enumerate(ranges):
        if 0 < value < min_value:
            min_value = value
            min_index = index
    min_dist = ranges[index]
    min_angle = thetas[index]

    return min_dist, min_angle


def cross_product(v1, v2):  #two vectors of length 3, corresponding to the x, y, and z components of the input vectors
    """Compute the Cross Product between two vectors.

    Args:
        v1 (list): First vector of length 3.
        v2 (list): Second vector of length 3.

    Returns:
        list: The result of the cross product operation.
    """
    res = np.cross(v1, v2)
    # TODO: Compute the cross product.
    return res


robot = MBot()
setpoint = 0  # TODO: Pick your setpoint.
# TODO: Declare any other variables you might need here.

try:
    while True:
        # Read the latest lidar scan.
        ranges, thetas = robot.read_lidar()

        # TODO: (P1.2) Write code to follow the nearest wall here.
        # Hint: You should use the functions cross_product and find_min_dist.

        # Optionally, sleep for a bit before reading a new scan.
        time.sleep(0.1)
except:
    # Catch any exception, including the user quitting, and stop the robot.
    robot.stop()