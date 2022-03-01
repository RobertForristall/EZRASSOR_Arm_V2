import math
from tf import transformations

def euclidean_distance(x1, x2, y1, y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)

def calculate_heading(world_state):
    x1 = world_state.position['x']
    y1 = world_state.position['y']
    x2 = world_state.target_location.x
    y2 = world_state.target_location.y

    dy, dx = y2-y1, x2-x1

    new_heading = 180*(math.atan2(dy, dx))/math.pi

    if new_heading == 0:
        new_heading = 360 + new_heading

    return new_heading

def adjust_angle(heading, new_heading):
    angle_difference = new_heading - heading
    angle_difference = (angle_difference + 180) % 360 - 180
    return math.pi * angle_difference / 180

def quaternion_to_yaw(pose):
    quat = {
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    }
    euler = transformations.euler_from_quaternion(quat)
    return euler[2]

def angle_is_safe(angle, dist, buffer, scan, threshold):

    buffer_angle = math.atan(buffer / threshold)

    angle1 = angle - buffer_angle
    angle2 = angle + buffer_angle

    index1 = int((angle1 - scan.angle_min) / scan.angle_increment)
    index2 = int((angle2 - scan.angle_min) / scan.angle_increment)

    start = min(index1, index2)
    end = max(index1, index2)

    if start < 0 or end >= len(scan.ranges):
        return False

    for i in range(start, end):
        if (not math.isnan(scan.ranges[i])) and scan.ranges[i] <= dist:
            return False
    
    return True

def get_best_angle(world_state, buffer, scan, threshold):

    best_score = None
    best_angle = None

    for i in range(0, len(scan.ranges)):

        angle = scan.angle_min + i * scan.angle_increment

        if not angle_is_safe(angle, threshold, buffer, scan, threshold):
            continue

        new_heading_deg = calculate_heading(world_state)
        angle2goal_rad = adjust_angle(world_state.heading, new_heading_deg)

        score = abs(angle2goal_rad - angle)
        if best_score is None or score < best_score:
            best_score = score
            best_angle = angle

    return best_angle

def rel_to_abs(current_heading_deg, relative_heading_rad):

    relative_heading_deg = 180 * relative_heading_rad / math.pi

    abs_heading = relative_heading_deg + current_heading_deg

    if abs_heading < 0:
        abs_heading = abs_heading + 360

    return abs_heading