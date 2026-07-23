#!/usr/bin/env python3

from contextlib import nullcontext
import time

import cv2
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge, CvBridgeError
import torch
import torch.nn.functional as F
from transformers import CLIPSegForImageSegmentation, CLIPSegProcessor


class ClipSegNode(Node):
    """Segmenta imágenes ROS 2 mediante un prompt de texto usando CLIPSeg."""

    def __init__(self) -> None:
        super().__init__("clipseg_node")

        self.declare_parameter("input_topic", "/cam1/color/image_raw")
        self.declare_parameter("mask_topic", "/clipseg/mask")
        self.declare_parameter("probability_topic", "/clipseg/probability")
        self.declare_parameter("overlay_topic", "/clipseg/overlay")
        self.declare_parameter("prompt", "table")
        self.declare_parameter("threshold", 0.50)
        self.declare_parameter("max_rate_hz", 2.0)
        self.declare_parameter("model_id", "CIDAS/clipseg-rd64-refined")
        self.declare_parameter("use_amp", True)

        input_topic = str(self.get_parameter("input_topic").value)
        mask_topic = str(self.get_parameter("mask_topic").value)
        probability_topic = str(self.get_parameter("probability_topic").value)
        overlay_topic = str(self.get_parameter("overlay_topic").value)
        model_id = str(self.get_parameter("model_id").value)

        self.bridge = CvBridge()
        self.last_process_time = 0.0
        self.processed_frames = 0

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(
            f"Cargando {model_id} en {self.device.type.upper()}..."
        )

        self.processor = CLIPSegProcessor.from_pretrained(model_id)
        self.model = CLIPSegForImageSegmentation.from_pretrained(model_id)
        self.model.to(self.device)
        self.model.eval()

        num_params = sum(parameter.numel() for parameter in self.model.parameters())
        self.get_logger().info(
            f"Modelo cargado: {num_params:,} parámetros."
        )

        # depth=1 y QoS de sensores evitan acumular imágenes antiguas cuando
        # la inferencia es más lenta que la cámara.
        self.subscription = self.create_subscription(
            ImageMsg,
            input_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.mask_publisher = self.create_publisher(
            ImageMsg, mask_topic, qos_profile_sensor_data
        )
        self.probability_publisher = self.create_publisher(
            ImageMsg, probability_topic, qos_profile_sensor_data
        )
        self.overlay_publisher = self.create_publisher(
            ImageMsg, overlay_topic, qos_profile_sensor_data
        )

        self.get_logger().info(f"Suscrito a: {input_topic}")
        self.get_logger().info(f"Máscara: {mask_topic}")
        self.get_logger().info(f"Probabilidad: {probability_topic}")
        self.get_logger().info(f"Overlay: {overlay_topic}")

    def image_callback(self, message: ImageMsg) -> None:
        max_rate_hz = float(self.get_parameter("max_rate_hz").value)
        now = time.monotonic()

        if max_rate_hz > 0.0:
            minimum_interval = 1.0 / max_rate_hz
            if now - self.last_process_time < minimum_interval:
                return

        self.last_process_time = now

        prompt = str(self.get_parameter("prompt").value).strip()
        threshold = float(self.get_parameter("threshold").value)
        use_amp = bool(self.get_parameter("use_amp").value)

        if not prompt:
            self.get_logger().warning("El parámetro 'prompt' está vacío.")
            return

        if not 0.0 <= threshold <= 1.0:
            self.get_logger().error("'threshold' debe estar entre 0.0 y 1.0.")
            return

        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8"
            )
        except CvBridgeError as error:
            self.get_logger().error(f"No se pudo convertir la imagen: {error}")
            return

        height, width = frame_bgr.shape[:2]
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(frame_rgb)

        inputs = self.processor(
            text=[prompt],
            images=[pil_image],
            padding="max_length",
            return_tensors="pt",
        )
        inputs = {
            key: value.to(self.device)
            for key, value in inputs.items()
        }

        amp_context = (
            torch.autocast(device_type="cuda", dtype=torch.float16)
            if use_amp and self.device.type == "cuda"
            else nullcontext()
        )

        inference_start = time.perf_counter()

        try:
            with torch.inference_mode(), amp_context:
                outputs = self.model(**inputs)

                logits = outputs.logits
                if logits.ndim == 2:
                    logits = logits.unsqueeze(0)

                # CLIPSeg produce una máscara de resolución interna.
                # Se interpola antes de aplicar el umbral para recuperar
                # exactamente el tamaño de la imagen ROS original.
                logits = F.interpolate(
                    logits.unsqueeze(1),
                    size=(height, width),
                    mode="bilinear",
                    align_corners=False,
                ).squeeze(1)

                probability = torch.sigmoid(logits[0])

        except RuntimeError as error:
            self.get_logger().error(f"Error durante la inferencia: {error}")
            return

        probability_np = probability.float().cpu().numpy()
        probability_u8 = np.clip(
            probability_np * 255.0, 0, 255
        ).astype(np.uint8)
        mask_u8 = (probability_np >= threshold).astype(np.uint8) * 255

        overlay = frame_bgr.copy()
        selected = mask_u8 > 0
        if np.any(selected):
            highlight = np.zeros_like(frame_bgr)
            highlight[:, :, 1] = 255
            blended = cv2.addWeighted(frame_bgr, 0.55, highlight, 0.45, 0.0)
            overlay[selected] = blended[selected]

        cv2.putText(
            overlay,
            f"{prompt} | threshold={threshold:.2f}",
            (12, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        mask_message = self.bridge.cv2_to_imgmsg(mask_u8, encoding="mono8")
        probability_message = self.bridge.cv2_to_imgmsg(
            probability_u8, encoding="mono8"
        )
        overlay_message = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")

        # Conserva timestamp y frame_id de la cámara.
        mask_message.header = message.header
        probability_message.header = message.header
        overlay_message.header = message.header

        self.mask_publisher.publish(mask_message)
        self.probability_publisher.publish(probability_message)
        self.overlay_publisher.publish(overlay_message)

        self.processed_frames += 1
        if self.processed_frames % 30 == 0:
            elapsed_ms = (time.perf_counter() - inference_start) * 1000.0
            occupied_ratio = float(np.count_nonzero(selected)) / selected.size
            self.get_logger().info(
                f"Inferencia: {elapsed_ms:.1f} ms | "
                f"píxeles segmentados: {occupied_ratio:.1%}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClipSegNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
