"""Save one frame of the arm camera as PNG with brightness stats.

    python3 docs/perf/capture_arm_image.py --out /tmp/arm.png
"""
import argparse

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="/tmp/arm_cam.png")
    parser.add_argument("--topic", default="/arm_camera/image_raw")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("arm_image_capture")
    frames = []

    def on_image(message):
        image = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.width, -1)
        frames.append(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))

    node.create_subscription(Image, args.topic, on_image, qos_profile_sensor_data)
    for _ in range(300):
        rclpy.spin_once(node, timeout_sec=0.1)
        if frames:
            break
    if not frames:
        print("NO IMAGE", flush=True)
    else:
        cv2.imwrite(args.out, frames[0])
        print(f"saved {args.out} shape={frames[0].shape} "
              f"mean={frames[0].mean():.1f} std={frames[0].std():.1f}", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
