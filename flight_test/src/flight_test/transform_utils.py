import math

def euler_from_quaternion(q):
    """
    Convert a Quaternion to roll, pitch, yaw.
    """
    x, y, z, w = q.x, q.y, q.z, q.w

    # Calculate Roll
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Calculate Pitch
    sinp = 2.0 * (w*y - z*x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Calculate Yaw
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
