#!/usr/bin/env python3

from collections import deque
from pathlib import Path
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException,
)


class VLMapsDatasetRecorder(Node):

    def __init__(self):
        super().__init__("vlmaps_dataset_recorder")

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter(
            "output_dir",
            str(Path.home() / "vlmaps_data" / "warehouse_jackal_01"),
        )

        self.declare_parameter("rgb_topic", "/rgb")
        self.declare_parameter("depth_topic", "/depth")
        self.declare_parameter("camera_info_topic", "/camera_info")

        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("camera_frame", "Camera")

        # Dataset sampling rate used in continuous mode.
        self.declare_parameter("max_rate_hz", 2.0)

        # Acquisition mode:
        #   continuous -> original rate-limited behaviour
        #   trigger    -> save exactly one RGB-D sample per request
        self.declare_parameter("capture_mode", "continuous")

        # RGB/depth approximate synchronization.
        self.declare_parameter("sync_slop_s", 0.10)

        # Maximum WALL-CLOCK time that a frame may wait
        # for its exact TF to become available.
        self.declare_parameter("tf_wait_timeout_s", 5.0)

        # Maximum number of pending RGB-D pairs.
        self.declare_parameter("tf_queue_size", 50)

        self.output_dir = Path(
            self.get_parameter("output_dir").value
        ).expanduser()

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter(
            "camera_info_topic"
        ).value

        self.global_frame = self.get_parameter("global_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value

        self.max_rate_hz = float(
            self.get_parameter("max_rate_hz").value
        )

        self.capture_mode = str(
            self.get_parameter("capture_mode").value
        ).strip().lower()

        if self.capture_mode not in (
            "continuous",
            "trigger",
        ):
            raise RuntimeError(
                "capture_mode must be 'continuous' or 'trigger'"
            )

        self.sync_slop_s = float(
            self.get_parameter("sync_slop_s").value
        )

        self.tf_wait_timeout_s = float(
            self.get_parameter("tf_wait_timeout_s").value
        )

        self.tf_queue_size = int(
            self.get_parameter("tf_queue_size").value
        )

        # ---------------------------------------------------------
        # Dataset directories
        # ---------------------------------------------------------

        self.rgb_dir = self.output_dir / "rgb"
        self.depth_dir = self.output_dir / "depth"

        self.rgb_dir.mkdir(parents=True, exist_ok=True)
        self.depth_dir.mkdir(parents=True, exist_ok=True)

        self.pose_path = self.output_dir / "poses.txt"
        self.timestamps_path = self.output_dir / "timestamps.txt"
        self.camera_info_path = self.output_dir / "camera_info.txt"

        existing_rgb = list(self.rgb_dir.glob("*.png"))
        existing_depth = list(self.depth_dir.glob("*.npy"))

        if (
            existing_rgb
            or existing_depth
            or self.pose_path.exists()
            or self.timestamps_path.exists()
        ):
            raise RuntimeError(
                f"Dataset directory is not empty: {self.output_dir}\n"
                "Use a new output_dir or remove the previous test."
            )

        self.pose_file = open(
            self.pose_path,
            "w",
            buffering=1,
        )

        self.timestamps_file = open(
            self.timestamps_path,
            "w",
            buffering=1,
        )

        self.timestamps_file.write(
            "# index "
            "rgb_sec rgb_nsec "
            "depth_sec depth_nsec "
            "tf_sec tf_nsec "
            "rgb_depth_delta_s\n"
        )

        # ---------------------------------------------------------
        # Helpers
        # ---------------------------------------------------------

        self.bridge = CvBridge()

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.camera_info_saved = False

        # Pending items:
        #
        # (rgb_msg, depth_msg, wall_time_when_queued)
        #
        self.pending_frames = deque()

        self.frame_index = 0
        self.last_queued_stamp_ns = None

        # Triggered-acquisition state.
        self.capture_requested = False
        self.manual_capture_inflight = False

        # ---------------------------------------------------------
        # CameraInfo subscription
        # ---------------------------------------------------------

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.capture_service = self.create_service(
            Trigger,
            "~/capture",
            self.capture_callback,
        )

        capture_status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.saved_frame_count_publisher = self.create_publisher(
            UInt32,
            "~/saved_frame_count",
            capture_status_qos,
        )

        initial_count = UInt32()
        initial_count.data = self.frame_index

        self.saved_frame_count_publisher.publish(
            initial_count
        )

        # ---------------------------------------------------------
        # RGB + Depth synchronization
        # ---------------------------------------------------------

        self.rgb_sub = Subscriber(
            self,
            Image,
            self.rgb_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self.depth_sub = Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=30,
            slop=self.sync_slop_s,
            allow_headerless=False,
        )

        self.sync.registerCallback(
            self.rgb_depth_callback
        )

        # ---------------------------------------------------------
        # TF-processing timer
        #
        # IMPORTANT:
        # We do NOT block inside rgb_depth_callback waiting for TF.
        # This allows TransformListener to continue receiving /tf.
        # ---------------------------------------------------------

        self.tf_timer = self.create_timer(
            0.05,
            self.process_pending_frames,
        )

        self.get_logger().info(
            f"Recording VLMaps dataset to: {self.output_dir}"
        )

        self.get_logger().info(
            f"RGB: {self.rgb_topic}"
        )

        self.get_logger().info(
            f"Depth: {self.depth_topic}"
        )

        self.get_logger().info(
            f"TF: {self.global_frame} -> {self.camera_frame}"
        )

        self.get_logger().info(
            f"Maximum save rate: {self.max_rate_hz:.2f} Hz"
        )

        self.get_logger().info(
            f"Capture mode: {self.capture_mode}"
        )

        if self.capture_mode == "trigger":
            self.get_logger().info(
                "Triggered acquisition enabled. "
                "Service: ~/capture"
            )

        self.get_logger().info(
            f"TF wait timeout: {self.tf_wait_timeout_s:.2f} s"
        )

    # -------------------------------------------------------------
    # Triggered acquisition
    # -------------------------------------------------------------

    def capture_callback(
        self,
        request,
        response,
    ):

        del request

        if self.capture_mode != "trigger":

            response.success = False
            response.message = (
                "Recorder is not in trigger mode."
            )

            return response

        if (
            self.capture_requested
            or self.manual_capture_inflight
        ):

            response.success = False
            response.message = (
                "A capture is already pending."
            )

            return response

        self.capture_requested = True

        response.success = True
        response.message = (
            f"Capture armed for frame "
            f"{self.frame_index:06d}"
        )

        self.get_logger().info(
            response.message
        )

        return response

    # -------------------------------------------------------------
    # Camera calibration
    # -------------------------------------------------------------

    def camera_info_callback(self, msg):

        if self.camera_info_saved:
            return

        with open(self.camera_info_path, "w") as f:

            f.write(f"frame_id: {msg.header.frame_id}\n")
            f.write(f"width: {msg.width}\n")
            f.write(f"height: {msg.height}\n")

            f.write("K:\n")

            for row in range(3):
                values = msg.k[
                    row * 3:(row + 1) * 3
                ]

                f.write(
                    "  "
                    + " ".join(
                        f"{v:.10f}"
                        for v in values
                    )
                    + "\n"
                )

            f.write(
                f"distortion_model: "
                f"{msg.distortion_model}\n"
            )

            f.write(
                "D: "
                + " ".join(
                    f"{v:.10f}"
                    for v in msg.d
                )
                + "\n"
            )

        self.camera_info_saved = True

        self.get_logger().info(
            f"Camera calibration saved: "
            f"{self.camera_info_path}"
        )

    # -------------------------------------------------------------
    # RGB + Depth synchronized callback
    # -------------------------------------------------------------

    def rgb_depth_callback(
        self,
        rgb_msg,
        depth_msg,
    ):

        rgb_stamp_ns = (
            rgb_msg.header.stamp.sec * 1_000_000_000
            + rgb_msg.header.stamp.nanosec
        )

        # ---------------------------------------------------------
        # Trigger gate / continuous-mode rate limiter
        # ---------------------------------------------------------

        if self.capture_mode == "trigger":

            if (
                not self.capture_requested
                or self.manual_capture_inflight
            ):
                return

            # Consume exactly one synchronized RGB-D pair.
            self.capture_requested = False
            self.manual_capture_inflight = True

        if (
            self.capture_mode == "continuous"
            and self.max_rate_hz > 0.0
            and self.last_queued_stamp_ns is not None
        ):
            min_dt_ns = int(
                1_000_000_000
                / self.max_rate_hz
            )

            if (
                rgb_stamp_ns
                - self.last_queued_stamp_ns
                < min_dt_ns
            ):
                return

        self.last_queued_stamp_ns = rgb_stamp_ns

        # ---------------------------------------------------------
        # Queue frame instead of blocking while waiting for TF
        # ---------------------------------------------------------

        self.pending_frames.append(
            (
                rgb_msg,
                depth_msg,
                time.monotonic(),
            )
        )

        # Safety against an ever-growing queue.
        while (
            len(self.pending_frames)
            > self.tf_queue_size
        ):
            self.pending_frames.popleft()

            self.get_logger().warning(
                "TF queue full; dropped oldest RGB-D frame."
            )

    # -------------------------------------------------------------
    # Process queued frames when exact TF becomes available
    # -------------------------------------------------------------

    def process_pending_frames(self):

        if not self.pending_frames:
            return

        rgb_msg, depth_msg, queued_wall_time = (
            self.pending_frames[0]
        )

        image_time = Time.from_msg(
            rgb_msg.header.stamp
        )

        # ---------------------------------------------------------
        # IMPORTANT:
        # No timeout here.
        #
        # We try once and return immediately if TF is not ready.
        # That gives the executor time to process incoming /tf.
        # ---------------------------------------------------------

        try:

            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.camera_frame,
                image_time,
            )

        except TransformException as exc:

            waiting_s = (
                time.monotonic()
                - queued_wall_time
            )

            if waiting_s >= self.tf_wait_timeout_s:

                self.pending_frames.popleft()

                self.get_logger().warning(
                    "Dropping RGB-D frame after "
                    f"{waiting_s:.2f}s waiting for exact TF: "
                    f"{exc}"
                )

                if self.capture_mode == "trigger":
                    self.manual_capture_inflight = False
                    self.capture_requested = True

                    self.get_logger().warning(
                        "Triggered capture will retry on the "
                        "next synchronized RGB-D pair."
                    )

            return

        # Exact TF exists: remove this pair from queue.
        self.pending_frames.popleft()

        saved = self.save_frame(
            rgb_msg,
            depth_msg,
            tf,
        )

        if self.capture_mode == "trigger":

            self.manual_capture_inflight = False

            if not saved:
                self.capture_requested = True

                self.get_logger().warning(
                    "Triggered capture failed while saving; "
                    "retrying on the next RGB-D pair."
                )

    # -------------------------------------------------------------
    # Save a complete synchronized sample
    # -------------------------------------------------------------

    def save_frame(
        self,
        rgb_msg,
        depth_msg,
        tf,
    ):

        # ---------------------------------------------------------
        # Convert images
        # ---------------------------------------------------------

        try:

            rgb_bgr = self.bridge.imgmsg_to_cv2(
                rgb_msg,
                desired_encoding="bgr8",
            )

            depth = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="32FC1",
            )

        except Exception as exc:

            self.get_logger().error(
                f"Image conversion failed: {exc}"
            )

            return False

        depth = np.asarray(
            depth,
            dtype=np.float32,
        )

        name = f"{self.frame_index:06d}"

        rgb_path = (
            self.rgb_dir / f"{name}.png"
        )

        depth_path = (
            self.depth_dir / f"{name}.npy"
        )

        # ---------------------------------------------------------
        # RGB
        # ---------------------------------------------------------

        if not cv2.imwrite(
            str(rgb_path),
            rgb_bgr,
        ):

            self.get_logger().error(
                f"Could not save {rgb_path}"
            )

            return False

        # ---------------------------------------------------------
        # Depth
        # ---------------------------------------------------------

        np.save(
            depth_path,
            depth,
        )

        # ---------------------------------------------------------
        # Camera pose in odom:
        #
        # px py pz qx qy qz qw
        # ---------------------------------------------------------

        t = tf.transform.translation
        q = tf.transform.rotation

        self.pose_file.write(
            f"{t.x:.9f} "
            f"{t.y:.9f} "
            f"{t.z:.9f} "
            f"{q.x:.9f} "
            f"{q.y:.9f} "
            f"{q.z:.9f} "
            f"{q.w:.9f}\n"
        )

        rgb_stamp_ns = (
            rgb_msg.header.stamp.sec
            * 1_000_000_000
            + rgb_msg.header.stamp.nanosec
        )

        depth_stamp_ns = (
            depth_msg.header.stamp.sec
            * 1_000_000_000
            + depth_msg.header.stamp.nanosec
        )

        delta_s = abs(
            rgb_stamp_ns
            - depth_stamp_ns
        ) / 1e9

        self.timestamps_file.write(
            f"{self.frame_index:06d} "
            f"{rgb_msg.header.stamp.sec} "
            f"{rgb_msg.header.stamp.nanosec} "
            f"{depth_msg.header.stamp.sec} "
            f"{depth_msg.header.stamp.nanosec} "
            f"{tf.header.stamp.sec} "
            f"{tf.header.stamp.nanosec} "
            f"{delta_s:.9f}\n"
        )

        # ---------------------------------------------------------
        # Diagnostics
        # ---------------------------------------------------------

        valid = (
            np.isfinite(depth)
            & (depth > 0)
        )

        valid_ratio = float(
            valid.mean()
        )

        self.pose_file.flush()
        self.timestamps_file.flush()

        self.frame_index += 1

        saved_count = UInt32()
        saved_count.data = self.frame_index

        self.saved_frame_count_publisher.publish(
            saved_count
        )

        if (
            self.frame_index <= 5
            or self.frame_index % 10 == 0
        ):

            if valid.any():

                depth_min = float(
                    depth[valid].min()
                )

                depth_max = float(
                    depth[valid].max()
                )

            else:

                depth_min = float("nan")
                depth_max = float("nan")

            self.get_logger().info(
                f"Saved {name} | "
                f"sync={delta_s:.4f}s | "
                f"valid_depth="
                f"{100.0 * valid_ratio:.1f}% | "
                f"depth="
                f"[{depth_min:.2f}, "
                f"{depth_max:.2f}]m | "
                f"queue="
                f"{len(self.pending_frames)}"
            )

        return True

    # -------------------------------------------------------------

    def destroy_node(self):

        try:

            if not self.pose_file.closed:
                self.pose_file.close()

            if not self.timestamps_file.closed:
                self.timestamps_file.close()

        finally:
            super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = VLMapsDatasetRecorder()

    try:

        rclpy.spin(node)

    except (
        KeyboardInterrupt,
        ExternalShutdownException,
    ):
        pass

    finally:

        node.destroy_node()

        # Ctrl+C may already have shut ROS down.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
