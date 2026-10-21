#!/usr/bin/env python3

import argparse
import math
import os
import time

import rclpy
import yaml

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from simulation_interfaces.msg import EntityState
from simulation_interfaces.srv import (
    GetEntityState,
    SetEntityState,
    SetSimulationState,
)
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger


class MappingPoseRunner(Node):

    def __init__(
        self,
        positions_file,
        position_ids,
        robot_entity,
        settle_time_s,
        capture_timeout_s,
        pose_tolerance_m,
    ):
        super().__init__("mapping_pose_runner")

        self.positions_file = os.path.expanduser(
            positions_file
        )

        self.position_ids = position_ids
        self.robot_entity = robot_entity
        self.settle_time_s = settle_time_s
        self.capture_timeout_s = capture_timeout_s
        self.pose_tolerance_m = pose_tolerance_m

        self.positions = self.load_positions()

        self.simulation_state_client = self.create_client(
            SetSimulationState,
            "/set_simulation_state",
        )

        self.get_entity_state_client = self.create_client(
            GetEntityState,
            "/get_entity_state",
        )

        self.set_entity_state_client = self.create_client(
            SetEntityState,
            "/set_entity_state",
        )

        self.capture_client = self.create_client(
            Trigger,
            "/vlmaps_dataset_recorder/capture",
        )

        self.saved_frame_count = None

        capture_status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.saved_frame_count_subscription = (
            self.create_subscription(
                UInt32,
                "/vlmaps_dataset_recorder/saved_frame_count",
                self.saved_frame_count_callback,
                capture_status_qos,
            )
        )

    def saved_frame_count_callback(self, message):

        self.saved_frame_count = int(
            message.data
        )

    def wait_for_capture_status(
        self,
        timeout_s=5.0,
    ):

        start = time.monotonic()

        while (
            rclpy.ok()
            and self.saved_frame_count is None
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            if (
                time.monotonic() - start
                > timeout_s
            ):
                raise RuntimeError(
                    "No saved_frame_count received"
                )

    def load_positions(self):

        with open(
            self.positions_file,
            "r",
            encoding="utf-8",
        ) as file:
            data = yaml.safe_load(file) or {}

        positions = data.get("positions", {})

        required = (
            "x",
            "y",
            "z",
            "qx",
            "qy",
            "qz",
            "qw",
        )

        for position_id in self.position_ids:

            if position_id not in positions:
                raise RuntimeError(
                    f"Missing position: {position_id}"
                )

            missing = [
                key
                for key in required
                if key not in positions[position_id]
            ]

            if missing:
                raise RuntimeError(
                    f"{position_id} missing fields: "
                    f"{missing}"
                )

        return positions

    def wait_for_services(self):

        services = (
            (
                self.simulation_state_client,
                "/set_simulation_state",
            ),
            (
                self.get_entity_state_client,
                "/get_entity_state",
            ),
            (
                self.set_entity_state_client,
                "/set_entity_state",
            ),
            (
                self.capture_client,
                "/vlmaps_dataset_recorder/capture",
            ),
        )

        for client, name in services:

            self.get_logger().info(
                f"Waiting for {name}..."
            )

            if not client.wait_for_service(
                timeout_sec=20.0
            ):
                raise RuntimeError(
                    f"Service unavailable: {name}"
                )

    def call_service(
        self,
        client,
        request,
        timeout_s=20.0,
    ):

        future = client.call_async(request)

        start = time.monotonic()

        while rclpy.ok() and not future.done():

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            if (
                time.monotonic() - start
                > timeout_s
            ):
                raise RuntimeError(
                    "Service call timeout"
                )

        response = future.result()

        if response is None:
            raise RuntimeError(
                "Service returned no response"
            )

        return response

    def set_simulation_state(self, state):

        request = SetSimulationState.Request()
        request.state.state = state

        response = self.call_service(
            self.simulation_state_client,
            request,
        )

        if response.result.result not in (
            1,
            101,
        ):
            raise RuntimeError(
                "Could not change simulation state: "
                f"{response.result.error_message}"
            )

    def teleport(self, position_id):

        pose = self.positions[position_id]

        request = SetEntityState.Request()
        request.entity = self.robot_entity

        state = EntityState()
        state.header.frame_id = "world"

        state.pose.position.x = float(pose["x"])
        state.pose.position.y = float(pose["y"])
        state.pose.position.z = float(pose["z"])

        state.pose.orientation.x = float(
            pose["qx"]
        )
        state.pose.orientation.y = float(
            pose["qy"]
        )
        state.pose.orientation.z = float(
            pose["qz"]
        )
        state.pose.orientation.w = float(
            pose["qw"]
        )

        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0

        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

        state.acceleration.linear.x = 0.0
        state.acceleration.linear.y = 0.0
        state.acceleration.linear.z = 0.0

        state.acceleration.angular.x = 0.0
        state.acceleration.angular.y = 0.0
        state.acceleration.angular.z = 0.0

        request.state = state

        response = self.call_service(
            self.set_entity_state_client,
            request,
        )

        if response.result.result != 1:
            raise RuntimeError(
                f"Could not place robot at "
                f"{position_id}: "
                f"{response.result.error_message}"
            )

        self.get_logger().info(
            f"Teleported to {position_id} | "
            f"x={pose['x']:.3f}, "
            f"y={pose['y']:.3f}"
        )

    def verify_pose(self, position_id):

        request = GetEntityState.Request()
        request.entity = self.robot_entity

        response = self.call_service(
            self.get_entity_state_client,
            request,
        )

        if response.result.result != 1:
            raise RuntimeError(
                "Could not read Jackal state"
            )

        actual = response.state.pose.position
        target = self.positions[position_id]

        dx = actual.x - float(target["x"])
        dy = actual.y - float(target["y"])

        error_xy = math.hypot(
            dx,
            dy,
        )

        self.get_logger().info(
            f"{position_id} settled | "
            f"actual=({actual.x:.3f}, "
            f"{actual.y:.3f}) | "
            f"error_xy={error_xy:.3f} m"
        )

        if error_xy > self.pose_tolerance_m:
            raise RuntimeError(
                f"{position_id} pose error too large: "
                f"{error_xy:.3f} m"
            )

    def trigger_capture(self, position_id):

        self.wait_for_capture_status()

        baseline = self.saved_frame_count
        expected = baseline + 1

        request = Trigger.Request()

        response = self.call_service(
            self.capture_client,
            request,
        )

        if not response.success:
            raise RuntimeError(
                f"Capture failed for {position_id}: "
                f"{response.message}"
            )

        self.get_logger().info(
            f"{position_id}: {response.message}"
        )

        start = time.monotonic()

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            if self.saved_frame_count == expected:

                self.get_logger().info(
                    f"{position_id}: capture confirmed | "
                    f"saved_frame_count="
                    f"{self.saved_frame_count}"
                )

                return

            if (
                self.saved_frame_count is not None
                and self.saved_frame_count > expected
            ):
                raise RuntimeError(
                    "Unexpected capture count jump: "
                    f"{baseline} -> "
                    f"{self.saved_frame_count}"
                )

            if (
                time.monotonic() - start
                > self.capture_timeout_s
            ):
                raise RuntimeError(
                    f"Timeout waiting for capture "
                    f"confirmation at {position_id}"
                )

    def run(self):

        self.wait_for_services()

        self.get_logger().info(
            "Sequence: "
            + " -> ".join(self.position_ids)
        )

        for position_id in self.position_ids:

            self.get_logger().info(
                "================================"
            )

            self.get_logger().info(
                f"START {position_id}"
            )

            # PAUSED
            self.set_simulation_state(2)

            self.teleport(
                position_id
            )

            # PLAYING
            self.set_simulation_state(1)

            self.get_logger().info(
                f"Settling for "
                f"{self.settle_time_s:.1f} s..."
            )

            time.sleep(
                self.settle_time_s
            )

            self.verify_pose(
                position_id
            )

            self.trigger_capture(
                position_id
            )

        self.get_logger().info(
            "================================"
        )

        self.get_logger().info(
            "3-pose mapping test completed"
        )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--positions-file",
        default=(
            "~/indoor/evaluation/config/"
            "multitarget_positions.yaml"
        ),
    )

    parser.add_argument(
        "--positions",
        nargs="+",
        default=[
            "P1",
            "P2",
            "P3",
        ],
    )

    parser.add_argument(
        "--robot-entity",
        default="/World/jackal_basic/base_link",
    )

    parser.add_argument(
        "--settle-time",
        type=float,
        default=3.0,
    )

    parser.add_argument(
        "--capture-timeout",
        type=float,
        default=10.0,
    )

    parser.add_argument(
        "--pose-tolerance",
        type=float,
        default=0.15,
    )

    args = parser.parse_args()

    rclpy.init()

    node = MappingPoseRunner(
        positions_file=args.positions_file,
        position_ids=args.positions,
        robot_entity=args.robot_entity,
        settle_time_s=args.settle_time,
        capture_timeout_s=args.capture_timeout,
        pose_tolerance_m=args.pose_tolerance,
    )

    try:
        node.run()
    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
