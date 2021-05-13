import math

# Takes in a quaternion in the form w, x, y, z, and returns pitch, roll, and yaw euler angles in degrees
def quat_to_euler(q0, q1, q2, q3):
    q0, q1, q2, q3 = normalize_quat(q0, q1, q2, q3)
    roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2))
    pitch = math.asin(2 * (q0 * q2 - q3 * q1))
    yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 ** 2 + q3 ** 2))

    pitch = math.degrees(-pitch)
    roll = math.degrees(roll)
    yaw = math.degrees(-yaw)

    return pitch, roll, yaw


# Normalizes a quaternion so the euclidean distance <= 1
def normalize_quat(q0, q1, q2, q3):
    d = math.sqrt(q0 ** 2 + q1 ** 2 + q2 ** 2 + q3 ** 2)
    if d == 0:
        print(f"Error. q0 = {q0}, q1 = {q1}, q2 = {q2}, q3 = {q3}")
        return q0, q1, q2, q3
    return q0 / d, q1 / d, q2 / d, q3 / d
