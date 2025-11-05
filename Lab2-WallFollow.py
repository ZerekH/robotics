import time
import numpy as np
from mbot_bridge.api import MBot


def find_min_dist(ranges, thetas):
    ranges = np.array(ranges)
    thetas = np.array(thetas)
    valid_idx = ranges > 0
    valid_ranges = ranges[valid_idx]
    valid_thetas = thetas[valid_idx]
    if len(valid_ranges) == 0:
        return None, None
    min_idx = np.argmin(valid_ranges)
    return valid_ranges[min_idx], valid_thetas[min_idx]


# ----------------------------
# Control parameters
# ----------------------------
WALL_SETPOINT = 0.3       # target distance from wall (m)
OBSTACLE_THRESHOLD = 0.25  # distance to consider obstacle (m)
SPEED = 0.2               # base tangential speed (m/s)
KP = 0.8                  # proportional gain for wall distance correction
SCAN_ANGLE = np.radians(60)

robot = MBot()
state = "SEARCHING"

print("Omnidirectional Wall Follower started!")
print(f"Wall setpoint: {WALL_SETPOINT}m\n")


def get_sector_distance(ranges, thetas, center_angle, width):
    """Compute average distance in a sector for front/side checking."""
    mask = (thetas > center_angle - width / 2) & (thetas < center_angle + width / 2)
    valid = ranges[mask]
    valid = valid[valid > 0]
    return np.mean(valid) if len(valid) > 0 else np.inf


try:
    while True:
        ranges, thetas = robot.read_lidar()
        ranges = np.array(ranges)
        thetas = np.array(thetas)

        if len(ranges) == 0:
            print("No LIDAR data")
            robot.drive(0, 0, 0)
            time.sleep(0.1)
            continue

        dist_to_wall, angle_to_wall = find_min_dist(ranges, thetas)
        if dist_to_wall is None:
            print("No valid wall detected")
            robot.drive(0, 0, 0)
            time.sleep(0.1)
            continue

        # Average front distance for obstacle detection
        front_dist = get_sector_distance(ranges, thetas, 0, SCAN_ANGLE)

        # ====================
        # STATE MACHINE LOGIC
        # ====================
        if state == "SEARCHING":
            if dist_to_wall < 1.0:
                print("Wall found! Switching to FOLLOWING.")
                state = "FOLLOWING"
            else:
                robot.drive(SPEED, 0, 0)
                print("Searching for wall...")
                time.sleep(0.1)
                continue

        elif state == "FOLLOWING":
            distance_error = dist_to_wall - WALL_SETPOINT
            front_close = front_dist < OBSTACLE_THRESHOLD

            if front_close:
                print("Obstacle ahead! Switching to AVOIDING.")
                state = "AVOIDING"
                continue

            # Compute wall tangent vector (parallel to wall)
            tangent_angle = angle_to_wall + np.pi / 2

            # Move tangentially while correcting distance perpendicular to wall
            vx = SPEED * np.cos(tangent_angle) - KP * distance_error * np.cos(angle_to_wall)
            vy = SPEED * np.sin(tangent_angle) - KP * distance_error * np.sin(angle_to_wall)

            robot.drive(vx, vy, 0)
            print(f"Following wall: dist={dist_to_wall:.2f}, err={distance_error:+.2f}, vx={vx:.2f}, vy={vy:.2f}")

        elif state == "AVOIDING":
            vx = 0.1
            vy = SPEED
            if front_dist > 0.5:
                print("Obstacle cleared. Returning to FOLLOWING.")
                state = "FOLLOWING"
            robot.drive(vx, vy, 0)
            print("Avoiding obstacle...")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping robot...")
    robot.stop()
except Exception as e:
    print(f"\nError: {e}")
    robot.stop()
