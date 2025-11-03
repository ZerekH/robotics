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


def cross_product(v1, v2):
    return np.cross(v1, v2)


# ============================
# WALL FOLLOWING CONTROL LOGIC
# ============================

class WallFollower:
    def __init__(self):
        self.robot = MBot()

        # Parameters
        self.WALL_SETPOINT = 0.3
        self.OBSTACLE_THRESHOLD = 0.25
        self.SPEED = 0.25
        self.Kp = 1.0  # proportional gain for wall distance
        self.SCAN_ANGLE = np.radians(60)  # field of view for front checking

        # State
        self.state = "SEARCHING"
        print("Omnidirectional Wall Follower initialized!")

    def get_lidar_data(self):
        ranges, thetas = self.robot.read_lidar()
        return np.array(ranges), np.array(thetas)

    def get_sector_distance(self, ranges, thetas, center_angle, width):
        """Compute average distance in a sector (used for front/side checking)."""
        mask = (thetas > center_angle - width / 2) & (thetas < center_angle + width / 2)
        valid = ranges[mask]
        valid = valid[valid > 0]
        return np.mean(valid) if len(valid) > 0 else np.inf

    def control(self):
        ranges, thetas = self.get_lidar_data()
        if len(ranges) == 0:
            print("No LIDAR data")
            self.robot.drive(0, 0, 0)
            return

        # Find closest object (potential wall)
        dist_to_wall, angle_to_wall = find_min_dist(ranges, thetas)

        if dist_to_wall is None:
            print("No valid wall detected.")
            self.robot.drive(0, 0, 0)
            return

        # Check distances in front and side
        front_dist = self.get_sector_distance(ranges, thetas, 0, self.SCAN_ANGLE)
        side_dist = self.get_sector_distance(ranges, thetas, np.pi / 2, self.SCAN_ANGLE)

        # --- STATE LOGIC ---

        # 1. SEARCHING: move forward until a wall is detected
        if self.state == "SEARCHING":
            if dist_to_wall < 1.0:
                print("Wall found! Switching to ALIGNING.")
                self.state = "ALIGNING"
            self.robot.drive(self.SPEED, 0, 0)
            return

        # 2. ALIGNING: adjust distance and orientation to get to setpoint
        if self.state == "ALIGNING":
            distance_error = dist_to_wall - self.WALL_SETPOINT
            adjust = self.Kp * distance_error

            # Move sideways toward/away from wall
            vx = self.SPEED * 0.5
            vy = -adjust  # negative moves toward wall if too far
            if abs(distance_error) < 0.05:
                print("Aligned with wall. Switching to FOLLOWING.")
                self.state = "FOLLOWING"
            self.robot.drive(vx, vy, 0)
            return

        # 3. FOLLOWING: move parallel to wall while maintaining distance
        if self.state == "FOLLOWING":
            distance_error = dist_to_wall - self.WALL_SETPOINT
            front_close = front_dist < self.OBSTACLE_THRESHOLD

            # If obstacle directly ahead, switch to avoiding
            if front_close:
                print("Obstacle ahead! Switching to AVOIDING.")
                self.state = "AVOIDING"
                return

            # Maintain distance using lateral correction
            vy = -self.Kp * distance_error
            vx = self.SPEED
            self.robot.drive(vx, vy, 0)
            print(f"Following wall: dist={dist_to_wall:.2f} err={distance_error:+.2f}")
            return

        # 4. AVOIDING: steer around obstacle, then resume following
        if self.state == "AVOIDING":
            # Turn left (or right) around the obstacle
            vx = 0.1
            vy = self.SPEED  # move sideways to avoid
            if front_dist > 0.5:
                print("Obstacle cleared. Returning to FOLLOWING.")
                self.state = "FOLLOWING"
            self.robot.drive(vx, vy, 0)
            return

    def run(self):
        print("Starting wall following loop. Press Ctrl+C to stop.")
        try:
            while True:
                self.control()
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Stopping robot.")
            self.robot.stop()
        except Exception as e:
            print(f"Error: {e}")
            self.robot.stop()


if __name__ == "__main__":
    bot = WallFollower()
    bot.run()
