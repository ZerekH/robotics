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
    ranges = np.array(ranges)
    thetas = np.array(thetas)
    
    # Filter out invalid readings (0 or negative)
    valid_idx = ranges > 0
    valid_ranges = ranges[valid_idx]
    valid_thetas = thetas[valid_idx]
    
    if len(valid_ranges) == 0:
        return None, None
    
    # Find minimum distance
    min_idx = np.argmin(valid_ranges)
    min_dist = valid_ranges[min_idx]
    min_angle = valid_thetas[min_idx]
    
    return min_dist, min_angle


def cross_product(v1, v2):
    """Compute the Cross Product between two vectors.

    Args:
        v1 (list): First vector of length 3.
        v2 (list): Second vector of length 3.

    Returns:
        list: The result of the cross product operation.
    """
    res = np.cross(v1, v2)
    return res


robot = MBot()

# Control parameters
WALL_SETPOINT = 0.3      # Target distance from wall (meters)
SPEED = 0.3            # Movement speed
OBSTACLE_THRESHOLD = 0.25  # Distance to consider obstacle (meters)

print("Omnidirectional Wall Follower started!")
print(f"Wall setpoint: {WALL_SETPOINT}m")
print("Robot will follow the nearest wall using cross product navigation.")
print("Press Ctrl+C to stop.\n")

try:
    while True:
        # Read the latest lidar scan
        ranges, thetas = robot.read_lidar()
        
        # Find the nearest wall (closest distance)
        dist_to_wall, angle_to_wall = find_min_dist(ranges, thetas)
        
        if dist_to_wall is None:
            # No valid readings - stop
            robot.drive(0, 0, 0)
            print("No wall detected")
            time.sleep(0.1)
            continue
        
        # Create vector pointing to the wall (in 2D, z=0)
        wall_vector = np.array([
            dist_to_wall * np.cos(angle_to_wall),
            dist_to_wall * np.sin(angle_to_wall),
            0
        ])
        
        # Create forward direction vector (robot's forward is x-axis)
        forward_vector = np.array([1, 0, 0])
        
        # Compute cross product to get direction parallel to wall
        # The cross product gives us a vector perpendicular to both:
        # - perpendicular to wall direction = parallel to wall
        # - perpendicular to forward = side direction
        cross = cross_product(wall_vector, forward_vector)
        
        # The z-component tells us which way to move to follow the wall
        # Positive z = turn/move left, Negative z = turn/move right
        direction = np.sign(cross[2])
        
        # Calculate distance error from setpoint
        distance_error = dist_to_wall - WALL_SETPOINT
        
        # Decide movement based on distance to wall
        if dist_to_wall < OBSTACLE_THRESHOLD:
            # Too close to wall - move away
            vx = -SPEED * np.cos(angle_to_wall)
            vy = -SPEED * np.sin(angle_to_wall)
            status = "Moving away"
        elif distance_error > 0.1:
            # Too far from wall - move toward it while moving parallel
            vx = SPEED * np.cos(angle_to_wall) * 0.5 + SPEED * direction * 0.5
            vy = SPEED * np.sin(angle_to_wall) * 0.5 + SPEED * direction * 0.5
            status = "Moving toward wall"
        else:

            print('good distance')
            robot.stop()
            # Good distance - move parallel to wall
            # Use cross product direction to determine parallel movement
            vx = SPEED * direction
            vy = SPEED * direction * np.sign(np.sin(angle_to_wall))
            status = "Following wall"
        # Send drive command
        robot.drive(vx, vy, 0)
        
        # Print status
        angle_deg = np.degrees(angle_to_wall)
        print(f"{status}: {dist_to_wall:.2f}m at {angle_deg:+.1f}° | "
              f"Cross: {cross[2]:.2f} | vx: {vx:.2f}, vy: {vy:.2f}")
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping robot...")
    robot.stop()
except Exception as e:
    print(f"\nError: {e}")
    robot.stop()