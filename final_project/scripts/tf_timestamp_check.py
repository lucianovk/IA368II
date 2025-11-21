#!/usr/bin/env python3
"""
TF/Map timestamp sanity checker.

Collects a short window of TF and map messages and reports:
- odom->base_link TF publish rate
- latest TF stamps for odom->base_link and map->odom (if present)
- latest map header stamp

Usage:
  ./scripts/tf_timestamp_check.py [--duration 5.0] [--base_frame base_link] [--odom_frame odom] [--map_frame map]
"""

import argparse
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def fmt_stamp(stamp) -> str:
    return f"{stamp.sec}.{stamp.nanosec:09d}"


class TFDump(Node):
    def __init__(self, base_frame: str, odom_frame: str, map_frame: str):
        super().__init__('tf_timestamp_check')
        self.base_frame = base_frame
        self.odom_frame = odom_frame
        self.map_frame = map_frame

        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 50)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)

        self.tf_count = 0
        self.tf_first_time = None
        self.tf_last_time = None
        self.latest_odom_base = None
        self.latest_map_odom = None
        self.latest_map_msg = None

    def tf_callback(self, msg: TFMessage):
        now = self.get_clock().now()
        if self.tf_first_time is None:
            self.tf_first_time = now
        self.tf_last_time = now

        for tf in msg.transforms:
            parent = tf.header.frame_id
            child = tf.child_frame_id
            if parent == self.odom_frame and child == self.base_frame:
                self.tf_count += 1
                self.latest_odom_base = tf.header.stamp
            if parent == self.map_frame and child == self.odom_frame:
                self.latest_map_odom = tf.header.stamp

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map_msg = msg.header.stamp

    def summary(self, duration_sec: float):
        print("=== TF timestamp check ===")
        if self.tf_first_time and self.tf_last_time:
            elapsed = (self.tf_last_time - self.tf_first_time).nanoseconds * 1e-9
            elapsed = max(elapsed, 1e-6)
            rate = self.tf_count / elapsed
            print(f"odom->{self.base_frame} TF rate: {rate:.2f} Hz over {elapsed:.2f}s")
        else:
            print("No TF messages received.")

        if self.latest_odom_base:
            print(f"Latest odom->{self.base_frame} TF stamp: {fmt_stamp(self.latest_odom_base)}")
        else:
            print(f"No odom->{self.base_frame} TF seen.")

        if self.latest_map_odom:
            print(f"Latest map->{self.odom_frame} TF stamp: {fmt_stamp(self.latest_map_odom)}")
        else:
            print(f"No map->{self.odom_frame} TF seen.")

        if self.latest_map_msg:
            print(f"Latest /map msg stamp: {fmt_stamp(self.latest_map_msg)}")
        else:
            print("No /map messages received.")

        print("\nIf map or TF stamps lag beyond your TF buffer, increase buffer duration or TF publish rate, or delay processing until TF catches up.")


def main():
    parser = argparse.ArgumentParser(description="TF and map timestamp inspector")
    parser.add_argument('--duration', type=float, default=5.0, help='Seconds to listen')
    parser.add_argument('--base_frame', default='base_link')
    parser.add_argument('--odom_frame', default='odom')
    parser.add_argument('--map_frame', default='map')
    args = parser.parse_args()

    rclpy.init()
    node = TFDump(args.base_frame, args.odom_frame, args.map_frame)

    end_time = time.time() + args.duration
    try:
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary(args.duration)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
