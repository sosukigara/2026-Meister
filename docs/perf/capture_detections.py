"""Wait for /detections and dump the first message as YAML.

    python3 docs/perf/capture_detections.py --out /tmp/det.yaml
"""
import argparse

import rclpy
from vision_msgs.msg import Detection2DArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="/tmp/detections.yaml")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("detection_capture")
    received = []

    def on_message(message):
        received.append(message)
        node.get_logger().info("received")

    node.create_subscription(Detection2DArray, "/detections", on_message, 10)
    print("subscriber ready", flush=True)
    for _ in range(300):
        rclpy.spin_once(node, timeout_sec=0.1)
        if received:
            break
    with open(args.out, "w", encoding="utf-8") as handle:
        handle.write(str(received[0]) if received else "NO MESSAGE\n")
    print(f"saved {len(received)} message(s) to {args.out}", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
