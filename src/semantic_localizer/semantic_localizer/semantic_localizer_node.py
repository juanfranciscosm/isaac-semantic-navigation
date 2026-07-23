#!/usr/bin/env python3

from collections import deque
import math

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener
from std_srvs.srv import Trigger

class SemanticLocalizerNode(Node):

    def __init__(self):
        super().__init__('semantic_localizer')

        # Tópicos de entrada
        self.declare_parameter('mask_topic', '/clipseg/mask')
        self.declare_parameter('depth_topic', '/depth')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('camera_tf_frame', 'Camera')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('standoff_distance_m', 0.80)
        self.declare_parameter('history_size', 5)
        self.declare_parameter('minimum_stable_samples', 3)
        self.declare_parameter('maximum_spread_m', 0.25)
        self.declare_parameter('history_reset_distance_m', 1.00)
        self.declare_parameter('lock_target_after_first_detection', True)


        # Parámetros de sincronización y filtrado
        self.declare_parameter('sync_slop', 1.0)
        self.declare_parameter('min_depth_m', 0.20)
        self.declare_parameter('max_depth_m', 10.0)
        self.declare_parameter('depth_band_m', 0.30)
        self.declare_parameter('min_component_area_px', 200)
        self.declare_parameter('min_valid_points', 100)

        self.mask_topic = str(
            self.get_parameter('mask_topic').value
        )
        self.depth_topic = str(
            self.get_parameter('depth_topic').value
        )
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value
        )

        self.camera_tf_frame = str(
            self.get_parameter('camera_tf_frame').value
        )

        self.base_frame = str(
            self.get_parameter('base_frame').value
        )

        self.standoff_distance_m = float(
            self.get_parameter('standoff_distance_m').value
        )

        history_size = int(
            self.get_parameter('history_size').value
        )

        self.minimum_stable_samples = int(
            self.get_parameter('minimum_stable_samples').value
        )

        self.maximum_spread_m = float(
            self.get_parameter('maximum_spread_m').value
        )

        self.history_reset_distance_m = float(
            self.get_parameter('history_reset_distance_m').value
        )

        self.target_frame = str(
            self.get_parameter('target_frame').value
        )

        self.sync_slop = float(
            self.get_parameter('sync_slop').value
        )
        self.min_depth_m = float(
            self.get_parameter('min_depth_m').value
        )
        self.max_depth_m = float(
            self.get_parameter('max_depth_m').value
        )
        self.depth_band_m = float(
            self.get_parameter('depth_band_m').value
        )
        self.min_component_area_px = int(
            self.get_parameter('min_component_area_px').value
        )
        self.min_valid_points = int(
            self.get_parameter('min_valid_points').value
        )
        self.lock_target_after_first_detection = bool(
            self.get_parameter(
                'lock_target_after_first_detection'
            ).value
        )

        self.bridge = CvBridge()
        self.camera_info = None

        self.position_history = deque(
            maxlen=history_size
        )

        self.locked_target_position = None

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # CameraInfo contiene fx, fy, cx y cy.
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        # Publicador del centroide 3D en el frame de la cámara.
        self.point_camera_publisher = self.create_publisher(
            PointStamped,
            '/semantic_object/point_camera',
            10
        )

        self.point_odom_publisher = self.create_publisher(
            PointStamped,
            '/semantic_object/point_odom',
            10
        )

        self.point_odom_stable_publisher = self.create_publisher(
            PointStamped,
            '/semantic_object/point_odom_stable',
            10
        )

        self.goal_pose_publisher = self.create_publisher(
            PoseStamped,
            '/semantic_object/goal_pose',
            10
        )

        self.reset_target_service = self.create_service(
            Trigger,
            '/semantic_object/reset_target',
            self.reset_target_callback
        )

        # Suscriptores sincronizados.
        self.mask_sub = Subscriber(
            self,
            Image,
            self.mask_topic,
            qos_profile=qos_profile_sensor_data
        )

        self.depth_sub = Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=qos_profile_sensor_data
        )

        self.synchronizer = ApproximateTimeSynchronizer(
            [self.mask_sub, self.depth_sub],
            queue_size=10,
            slop=self.sync_slop
        )

        self.synchronizer.registerCallback(
            self.synced_callback
        )

        self.get_logger().info(
            'semantic_localizer_node iniciado'
        )
        self.get_logger().info(
            f'Máscara: {self.mask_topic}'
        )
        self.get_logger().info(
            f'Profundidad: {self.depth_topic}'
        )
        self.get_logger().info(
            'Salida: /semantic_object/point_camera'
        )
        self.get_logger().info(
            'Salida transformada: /semantic_object/point_odom'
        )
        self.get_logger().info(
            'Meta de aproximación: /semantic_object/goal_pose'
        )

    def camera_info_callback(self, message):
        self.camera_info = message

    def convert_depth_to_meters(self, depth_message):
        depth = self.bridge.imgmsg_to_cv2(
            depth_message,
            desired_encoding='passthrough'
        )

        if depth_message.encoding == '32FC1':
            # 32FC1 normalmente ya está expresado en metros.
            return depth.astype(np.float32)

        if depth_message.encoding in ('16UC1', 'mono16'):
            # 16UC1 normalmente está expresado en milímetros.
            return depth.astype(np.float32) / 1000.0

        raise ValueError(
            'Codificación de profundidad no soportada: '
            f'{depth_message.encoding}'
        )

    def select_largest_component(self, binary_mask):
        """
        Separa regiones desconectadas de la máscara y devuelve
        únicamente la región más grande.
        """

        number_of_labels, labels, stats, _ = (
            cv2.connectedComponentsWithStats(
                binary_mask,
                connectivity=8
            )
        )

        # La etiqueta 0 siempre es el fondo.
        if number_of_labels <= 1:
            return None

        component_areas = stats[
            1:,
            cv2.CC_STAT_AREA
        ]

        largest_label = (
            int(np.argmax(component_areas)) + 1
        )

        largest_area = int(
            stats[
                largest_label,
                cv2.CC_STAT_AREA
            ]
        )

        if largest_area < self.min_component_area_px:
            return None

        return labels == largest_label
    
    def reset_target_callback(self, request, response):
        self.locked_target_position = None
        self.position_history.clear()

        response.success = True
        response.message = (
            'Objetivo desbloqueado. '
            'La próxima detección estable será seleccionada.'
        )

        self.get_logger().info(response.message)

        return response

    def synced_callback(self, mask_message, depth_message):
        if self.camera_info is None:
            self.get_logger().warning(
                'Esperando /camera_info'
            )
            return

        try:
            mask = self.bridge.imgmsg_to_cv2(
                mask_message,
                desired_encoding='mono8'
            )

            depth_m = self.convert_depth_to_meters(
                depth_message
            )

        except Exception as error:
            self.get_logger().error(
                f'Error convirtiendo imágenes: {error}'
            )
            return

        if mask.shape != depth_m.shape:
            self.get_logger().error(
                'La máscara y la profundidad tienen '
                f'tamaños diferentes: {mask.shape} y '
                f'{depth_m.shape}'
            )
            return

        # Convierte la máscara mono8 en una matriz binaria:
        # 0 = fondo, 1 = objeto.
        binary_mask = (
            mask > 0
        ).astype(np.uint8)

        # Elimina pequeños puntos aislados.
        kernel = np.ones(
            (3, 3),
            dtype=np.uint8
        )

        binary_mask = cv2.morphologyEx(
            binary_mask,
            cv2.MORPH_OPEN,
            kernel,
            iterations=1
        )

        component_mask = self.select_largest_component(
            binary_mask
        )

        if component_mask is None:
            self.get_logger().warning(
                'No se encontró una región segmentada válida'
            )
            return

        # np.where devuelve:
        # v = fila de la imagen
        # u = columna de la imagen
        v_pixels, u_pixels = np.where(
            component_mask
        )

        # Obtiene la profundidad correspondiente a cada píxel.
        z_values = depth_m[
            v_pixels,
            u_pixels
        ]

        # Elimina NaN, infinitos, ceros y distancias fuera de rango.
        valid_depth = (
            np.isfinite(z_values)
            & (z_values >= self.min_depth_m)
            & (z_values <= self.max_depth_m)
        )

        u_pixels = u_pixels[valid_depth]
        v_pixels = v_pixels[valid_depth]
        z_values = z_values[valid_depth]

        if len(z_values) < self.min_valid_points:
            self.get_logger().warning(
                'Muy pocos píxeles contienen profundidad válida: '
                f'{len(z_values)}'
            )
            return

        # Profundidad central del objeto.
        median_depth = float(
            np.median(z_values)
        )

        # Descarta puntos del fondo o de superficies alejadas.
        same_surface = (
            np.abs(z_values - median_depth)
            <= self.depth_band_m
        )

        u_pixels = u_pixels[same_surface]
        v_pixels = v_pixels[same_surface]
        z_values = z_values[same_surface]

        if len(z_values) < self.min_valid_points:
            self.get_logger().warning(
                'Muy pocos puntos después del filtrado: '
                f'{len(z_values)}'
            )
            return

        # Parámetros intrínsecos de la cámara.
        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])

        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().error(
                'CameraInfo contiene fx o fy inválidos'
            )
            return

        # Proyección inversa:
        #
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        # Z = profundidad
        x_values = (
            (u_pixels.astype(np.float32) - cx)
            * z_values
            / fx
        )

        y_values = (
            (v_pixels.astype(np.float32) - cy)
            * z_values
            / fy
        )

        object_points = np.column_stack(
            (
                x_values,
                y_values,
                z_values
            )
        )

        # La mediana es más robusta que el promedio.
        center_3d = np.median(
            object_points,
            axis=0
        )

        point_message = PointStamped()

        # Conservamos el frame y timestamp de profundidad.
        point_message.header.stamp = depth_message.header.stamp
        point_message.header.frame_id = self.camera_tf_frame

        point_message.point.x = float(
            center_3d[0]
        )
        point_message.point.y = float(
            center_3d[1]
        )
        point_message.point.z = float(
            center_3d[2]
        )

        self.point_camera_publisher.publish(
            point_message
        )

    
        point_for_tf = PointStamped()

        point_for_tf.header.stamp = depth_message.header.stamp
        point_for_tf.header.frame_id = self.camera_tf_frame

        point_for_tf.point.x = point_message.point.x
        point_for_tf.point.y = point_message.point.y
        point_for_tf.point.z = point_message.point.z

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_tf_frame,
                Time.from_msg(depth_message.header.stamp),
                timeout=Duration(seconds=0.5)
            )

            point_odom = do_transform_point(
                point_for_tf,
                transform
            )

            point_odom.header.frame_id = self.target_frame
            point_odom.header.stamp = depth_message.header.stamp

            self.point_odom_publisher.publish(
                point_odom
            )

            raw_position = np.array(
                [
                    point_odom.point.x,
                    point_odom.point.y,
                    point_odom.point.z
                ],
                dtype=np.float64
            )

            # Si ya existe un objetivo bloqueado, no permitimos
            # que otra caja cambie la posición seleccionada.
            if (
                self.lock_target_after_first_detection
                and self.locked_target_position is not None
            ):
                stable_position = (
                    self.locked_target_position.copy()
                )

                maximum_spread = 0.0

            else:
                # Todavía estamos buscando una detección estable.
                if len(self.position_history) > 0:
                    previous_position = (
                        self.position_history[-1]
                    )

                    jump_distance = float(
                        np.linalg.norm(
                            raw_position[:2]
                            - previous_position[:2]
                        )
                    )

                    if (
                        jump_distance
                        > self.history_reset_distance_m
                    ):
                        self.get_logger().warning(
                            'Salto grande en la detección: '
                            f'{jump_distance:.2f} m. '
                            'Reiniciando historial.'
                        )

                        self.position_history.clear()

                self.position_history.append(
                    raw_position
                )

                if (
                    len(self.position_history)
                    < self.minimum_stable_samples
                ):
                    self.get_logger().info(
                        'Acumulando detecciones: '
                        f'{len(self.position_history)}/'
                        f'{self.minimum_stable_samples}'
                    )
                    return

                history_array = np.vstack(
                    self.position_history
                )

                stable_position = np.median(
                    history_array,
                    axis=0
                )

                distances_to_center = np.linalg.norm(
                    history_array[:, :2]
                    - stable_position[:2],
                    axis=1
                )

                maximum_spread = float(
                    np.max(distances_to_center)
                )

                if (
                    maximum_spread
                    > self.maximum_spread_m
                ):
                    self.get_logger().warning(
                        'Detección todavía inestable | '
                        f'dispersión={maximum_spread:.2f} m | '
                        f'máximo={self.maximum_spread_m:.2f} m'
                    )
                    return

                if self.lock_target_after_first_detection:
                    self.locked_target_position = (
                        stable_position.copy()
                    )

                    self.position_history.clear()

                    self.get_logger().info(
                        'OBJETIVO BLOQUEADO | '
                        f'X={stable_position[0]:.3f}, '
                        f'Y={stable_position[1]:.3f}, '
                        f'Z={stable_position[2]:.3f}'
                    )
                    
            stable_point = PointStamped()
            
            stable_point.header.frame_id = self.target_frame
            stable_point.header.stamp = depth_message.header.stamp

            stable_point.point.x = float(
                stable_position[0]
            )
            stable_point.point.y = float(
                stable_position[1]
            )
            stable_point.point.z = float(
                stable_position[2]
            )

            self.point_odom_stable_publisher.publish(
                stable_point
            )

            self.get_logger().info(
                'Centroide estable | '
                f'X={stable_position[0]:.3f} m, '
                f'Y={stable_position[1]:.3f} m, '
                f'Z={stable_position[2]:.3f} m | '
                f'muestras={len(self.position_history)} | '
                f'dispersión={maximum_spread:.3f} m'
            )

            # Obtiene la posición del robot en el mismo frame "odom".
            robot_transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                Time.from_msg(depth_message.header.stamp),
                timeout=Duration(seconds=0.5)
            )

            robot_x = float(
                robot_transform.transform.translation.x
            )

            robot_y = float(
                robot_transform.transform.translation.y
            )

            object_x = float(
                stable_position[0]
            )

            object_y = float(
                stable_position[1]
            )

            # Vector desde el robot hacia el objeto.
            delta_x = object_x - robot_x
            delta_y = object_y - robot_y

            distance_to_object = math.hypot(
                delta_x,
                delta_y
            )

            if distance_to_object <= (
                self.standoff_distance_m + 0.05
            ):
                self.get_logger().info(
                    'El robot ya se encuentra cerca del objeto: '
                    f'{distance_to_object:.2f} m'
                )
                return

            # Calcula un punto situado antes del objeto.
            goal_x = (
                object_x
                - self.standoff_distance_m
                * delta_x
                / distance_to_object
            )

            goal_y = (
                object_y
                - self.standoff_distance_m
                * delta_y
                / distance_to_object
            )

            # Ángulo que permite mirar desde la meta hacia el objeto.
            goal_yaw = math.atan2(
                object_y - goal_y,
                object_x - goal_x
            )

            goal_pose = PoseStamped()

            goal_pose.header.frame_id = self.target_frame
            goal_pose.header.stamp = depth_message.header.stamp

            goal_pose.pose.position.x = goal_x
            goal_pose.pose.position.y = goal_y
            goal_pose.pose.position.z = 0.0

            # Conversión de yaw a quaternion.
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = math.sin(
                goal_yaw / 2.0
            )
            goal_pose.pose.orientation.w = math.cos(
                goal_yaw / 2.0
            )

            self.goal_pose_publisher.publish(
                goal_pose
            )

            self.get_logger().info(
                'Meta calculada | '
                f'robot=({robot_x:.2f}, {robot_y:.2f}) | '
                f'objeto=({object_x:.2f}, {object_y:.2f}) | '
                f'meta=({goal_x:.2f}, {goal_y:.2f}) | '
                f'distancia={distance_to_object:.2f} m'
            )

            self.get_logger().info(
                'Centroide en odom | '
                f'X={point_odom.point.x:.3f} m, '
                f'Y={point_odom.point.y:.3f} m, '
                f'Z={point_odom.point.z:.3f} m'
            )

        except Exception as error:
            self.get_logger().warning(
                'No se pudo transformar '
                f'{self.camera_tf_frame} -> {self.target_frame}: '
                f'{error}'
            )

        self.get_logger().info(
            'Centroide 3D | '
            f'X={center_3d[0]:.3f} m, '
            f'Y={center_3d[1]:.3f} m, '
            f'Z={center_3d[2]:.3f} m | '
            f'puntos={len(object_points)} | '
            f'frame={depth_message.header.frame_id}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SemanticLocalizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()