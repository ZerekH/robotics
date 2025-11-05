import time
import numpy as np
from mbot_bridge.api import MBot

##############################
# Helper Functions
##############################

def avg_dist_in_sector(ranges, thetas, center_deg, width_deg=20):
    """
    Calculate average LiDAR range inside an angular window.
    center_deg: central angle in degrees (0 = front, 90 = left, 180 = rear, 270 = right)
    width_deg: +- window around the angle
    """
    center = np.radians(center_deg)
    width = np.radians(width_deg)

    ranges = np.array(ranges)
    thetas = np.array(thetas)

    mask = (thetas > center - width) & (thetas < center + width) & (ranges > 0)

    if np.sum(mask) == 0:
        return None

    return np.mean(ranges[mask])


##############################
# Wall Following Parameters
##############################

TARGET = 0.5     # target wall distance (meters)
THRESH = 0.15     # tolerance before declaring "wall lost"
TURN_RATE = 0.4   # rad/s yaw rotation
FWD_SPEED = 0.25  # forward velocity

##############################
# Init Robot
##############################

robot = MBot()
print("\nRight-wall following started!")
print(f"Target distance: {TARGET}m from wall\n")


##############################
# Main Control Loop
##############################

try:
    while True:
        ranges, thetas = robot.read_lidar()

        # Distances from key sectors
        right_dist = avg_dist_in_sector(ranges, thetas, 270, 25)
        front_dist = avg_dist_in_sector(ranges, thetas,   0, 15)

        # If no valid readings, freeze
        if right_dist is None:
            robot.drive(0, 0, 0)
            print("⚠️ No LiDAR data")
            time.sleep(0.1)
            continue

        # ✅ CASE 1: End of wall → right side opens up & front is clear
        if right_dist > TARGET + THRESH and (front_dist is None or front_dist > 0.8):
            print(f"↱ Wall ended — rotating RIGHT | right={right_dist:.2f}m")
            robot.drive(0, 0, -TURN_RATE)  # rotate right
            time.sleep(0.1)
            continue

        # ✅ CASE 2: Obstacle in front → turn left
        if front_dist is not None and front_dist < 0.4:
            print(f"⤴ Obstacle ahead — turning LEFT | front={front_dist:.2f}m")
            robot.drive(0, 0, TURN_RATE)
            time.sleep(0.1)
            continue

        # ✅ CASE 3: Normal wall following (P controller)
        error = TARGET - right_dist
        wz = -error * 2.0  # steer toward/away from wall

        print(f"➡ Following wall | right={right_dist:.2f}m front={front_dist}")
        robot.drive(FWD_SPEED, 0, wz)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopping robot...")
    robot.stop()

except Exception as e:
    print(f"\nError: {e}")
    robot.stop()
