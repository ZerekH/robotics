import time
import numpy as np
from mbot_bridge.api import MBot

def find_min_dist(ranges, thetas):
    ranges, thetas = np.array(ranges), np.array(thetas)
    valid = ranges > 0
    if not np.any(valid):
        return None, None
    idx = np.argmin(ranges[valid])
    return ranges[valid][idx], thetas[valid][idx]

def get_wall_vector(ranges, thetas, side='right', sector_angle=60):
    ranges, thetas = np.array(ranges), np.array(thetas)
    center = -np.pi/2 if side == 'right' else np.pi/2
    mask = (np.abs(thetas - center) < np.radians(sector_angle)/2) & (ranges > 0)
    if not np.any(mask):
        return None, None
    return np.mean(ranges[mask]), np.mean(thetas[mask])

def check_front_obstacle(ranges, thetas, threshold=0.4, width=30):
    ranges, thetas = np.array(ranges), np.array(thetas)
    mask = (np.abs(thetas) < np.radians(width)/2) & (ranges > 0)
    if not np.any(mask):
        return False, float('inf')
    min_dist = np.min(ranges[mask])
    return min_dist < threshold, min_dist

def cross_z(v1, v2):
    """Return z-component of 3D cross product for 2D vectors."""
    return v1[0]*v2[1] - v1[1]*v2[0]

robot = MBot()

# --- Parameters ---
WALL_SETPOINT = 0.35
FOLLOW_SIDE = 'right'
BASE_SPEED, MIN_SPEED, MAX_SPEED = 0.35, 0.1, 0.35
MAX_STRAFE = 0.3
KP_WALL, KP_ANGLE, OBSTACLE_GAIN = 1.5, 1.0, 2.0
OBSTACLE_THRESHOLD, CORNER_THRESHOLD = 0.4, 0.8
WALL_LOST_TIMEOUT = 1.2  # seconds before going into search mode

print(f"Omnidirectional wall following activated — following {FOLLOW_SIDE} wall @ {WALL_SETPOINT}m.")
print("Press Ctrl+C to stop.\n")

last_wall_seen = time.time()

try:
    while True:
        ranges, thetas = robot.read_lidar()
        min_dist, min_angle = find_min_dist(ranges, thetas)
        if min_dist is None:
            robot.stop()
            print("No valid LIDAR data.")
            time.sleep(0.1)
            continue

        wall_dist, wall_angle = get_wall_vector(ranges, thetas, side=FOLLOW_SIDE)
        has_front_obstacle, front_dist = check_front_obstacle(ranges, thetas, OBSTACLE_THRESHOLD)

        vx, vy, wz = BASE_SPEED, 0.0, 0.0

        # Predefined forward direction (always available)
        forward_dir = np.array([1.0, 0.0])

        # --- Forward speed control ---
        if has_front_obstacle:
            vx *= max(0.2, (front_dist - 0.2) / OBSTACLE_THRESHOLD)
            print(f"⚠️ Obstacle ahead at {front_dist:.2f}m — slowing down")

        # --- Wall following using cross product ---
        if wall_dist is not None:
            last_wall_seen = time.time()
            wall_error = wall_dist - WALL_SETPOINT
            wall_dir = np.array([np.cos(wall_angle), np.sin(wall_angle)])  # direction to wall

            # Cross product determines how angled wall is relative to robot
            wall_cross = cross_z(forward_dir, wall_dir)

            # Strafe based on distance from wall (toward/away)
            vy = (-1 if FOLLOW_SIDE == 'right' else 1) * KP_WALL * wall_error
            # Turn rate based on angular misalignment (cross product)
            wz = (-1 if FOLLOW_SIDE == 'right' else 1) * KP_ANGLE * wall_cross

            status = f"Wall {wall_dist:.2f}m @ {np.degrees(wall_angle):.0f}°"
        else:
            # Check how long since wall last seen
            time_since_seen = time.time() - last_wall_seen
            if time_since_seen > WALL_LOST_TIMEOUT:
                # Go into search mode
                wz = 0.6 if FOLLOW_SIDE == 'right' else -0.6
                vy = -0.1 if FOLLOW_SIDE == 'right' else 0.1
                vx = MIN_SPEED
                status = "🔍 Searching for wall..."
            else:
                status = f"Lost wall {time_since_seen:.1f}s ago — maintaining heading"

        # --- Obstacle avoidance via cross product ---
        if min_dist < OBSTACLE_THRESHOLD:
            obstacle_dir = np.array([np.cos(min_angle), np.sin(min_angle)])
            obs_cross = cross_z(forward_dir, obstacle_dir)
            # Rotate and strafe away from obstacle
            wz -= OBSTACLE_GAIN * obs_cross
            vy -= OBSTACLE_GAIN * 0.3 * np.cos(min_angle)
            if min_dist < 0.25:
                vx = MIN_SPEED
                print(f"🚨 Close obstacle {min_dist:.2f}m")

        # --- Corner handling ---
        if wall_dist is not None and wall_dist > CORNER_THRESHOLD:
            wz += -1.0 if FOLLOW_SIDE == 'right' else 1.0
            print("🔄 Corner detected!")

        # Clamp velocities
        vx = np.clip(vx, MIN_SPEED, MAX_SPEED)
        vy = np.clip(vy, -MAX_STRAFE, MAX_STRAFE)

        robot.drive(vx, vy, wz)
        print(f"{status} | Min {min_dist:.2f}m @ {np.degrees(min_angle):.0f}° | "
              f"Cmd vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping robot...")
    robot.stop()
except Exception as e:
    print(f"\nError: {e}")
    robot.stop()
