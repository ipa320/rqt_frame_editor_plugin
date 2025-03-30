import rclpy
import os

def can_transform(tf_buffer, target_frame, source_frame, time_):
    return tf_buffer.can_transform_core(target_frame, source_frame, time_)[0]


def wait_for_transform(tf_buffer, target_frame, source_frame, timeout):
    request_time = rclpy.Time.now()
    timeout_d = rclpy.duration.Duration(seconds=timeout)
    r = rclpy.Rate(100)
    while rclpy.Time.now() < request_time + timeout_d:
        # Try to get the most recent transform
        if can_transform(tf_buffer, target_frame, source_frame, rclpy.Time(0)):
            transform = tf_buffer.lookup_transform_core(
                target_frame, source_frame, rclpy.Time(0))
            # Success if it is newer than the initial time
            if transform.header.stamp > request_time:
                break
        r.sleep()
    else:
        raise RuntimeError('Transform timeout.')

