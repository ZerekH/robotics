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
    return v1[0]*v2[1] - v1[1]*v2[0]

robot = MBot()

WALL_SETPOINT = 0.35
FOLLOW_SIDE = 'right'
BASE_SPEED, MIN_SPEED, MAX_SPEED = 0.35, 0.1, 0.35
MAX_STRAFE = 0.3
KP_WALL, KP_ANGLE, OBSTACLE_GAIN = 1.5, 1.0, 2.0
OBSTACLE_THRESHOLD, CORNER_THRESHOLD = 0.4, 0.8
WALL_LOST_TIMEOUT = 1.2

last_wall = time.time()

try:
    while True:
        ranges, thetas = robot.read_lidar()
        min_dist, min_angle = find_min_dist(ranges, thetas)
        if min_dist is None:
            robot.stop()
            time.sleep(0.1)
            continue

        wall_dist, wall_angle = get_wall_vector(ranges, thetas, side=FOLLOW_SIDE)
        has_front_obstacle, front_dist = check_front_obstacle(ranges, thetas, OBSTACLE_THRESHOLD)

        vx, vy, wz = BASE_SPEED, 0.0, 0.0
        forward_dir = np.array([1.0, 0.0])

        if has_front_obstacle:
            vx *= max(0.2, (front_dist - 0.2) / OBSTACLE_THRESHOLD)

        if wall_dist is not None:
            last_wall = time.time()
            wall_error = wall_dist - WALL_SETPOINT
            wall_dir = np.array([np.cos(wall_angle), np.sin(wall_angle)])
            wall_cross = cross_z(forward_dir, wall_dir)
            vy = (-1 if FOLLOW_SIDE == 'right' else 1) * KP_WALL * wall_error
            wz = (-1 if FOLLOW_SIDE == 'right' else 1) * KP_ANGLE * wall_cross
        else:
            time_since_seen = time.time() - last_wall
            if time_since_seen > WALL_LOST_TIMEOUT:
                wz = 0.6 if FOLLOW_SIDE == 'right' else -0.6
                vy = -0.1 if FOLLOW_SIDE == 'right' else 0.1
                vx = MIN_SPEED
                print("Searching for wall")

        if min_dist < OBSTACLE_THRESHOLD:
            obstacle_dir = np.array([np.cos(min_angle), np.sin(min_angle)])
            obs_cross = cross_z(forward_dir, obstacle_dir)
            wz -= OBSTACLE_GAIN * obs_cross
            vy -= OBSTACLE_GAIN * 0.3 * np.cos(min_angle)
            if min_dist < 0.25:
                vx = MIN_SPEED
                print(f"Obstacle detected: {min_dist:.2f}")

        if wall_dist is not None and wall_dist > CORNER_THRESHOLD:
            wz += -1.0 if FOLLOW_SIDE == 'right' else 1.0

        vx = np.clip(vx, MIN_SPEED, MAX_SPEED)
        vy = np.clip(vy, -MAX_STRAFE, MAX_STRAFE)

        robot.drive(vx, vy, wz)
        time.sleep(0.1)

except KeyboardInterrupt:
    robot.stop()
