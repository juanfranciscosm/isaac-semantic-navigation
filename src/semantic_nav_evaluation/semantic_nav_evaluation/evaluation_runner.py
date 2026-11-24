#!/usr/bin/env python3

import csv
import json
import math
import os
import time
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
import yaml

from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.msg import (
    Parameter,
    ParameterType,
    ParameterValue,
)
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import Image
from simulation_interfaces.msg import EntityState
from simulation_interfaces.srv import (
    GetEntityState,
    SetEntityState,
    SetSimulationState,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger


class EvaluationRunner(Node):

    def __init__(self):
        super().__init__('evaluation_runner')

        # ---------------------------------------------------------
        # Parámetros
        # ---------------------------------------------------------

        self.declare_parameter('scenario', 'boxes_I0')
        self.declare_parameter('object_name', 'boxes')
        self.declare_parameter('condition', 'I0')
        self.declare_parameter(
            'position_filter',
            'ALL'
        )

        self.declare_parameter(
            'positions_file',
            '~/indoor/evaluation/config/boxes_positions.yaml'
        )

        self.declare_parameter(
            'ground_truth_file',
            ''
        )

        self.declare_parameter(
            'robot_entity',
            '/World/jackal_basic/base_link'
        )

        self.declare_parameter(
            'detection_timeout_s',
            40.0
        )

        self.declare_parameter(
            'navigation_timeout_s',
            120.0
        )

        self.declare_parameter(
            'navigation_wall_watchdog_s',
            900.0
        )

        self.declare_parameter(
            'settle_time_s',
            3.0
        )

        self.declare_parameter(
            'goal_region_tolerance_m',
            0.25
        )

        self.declare_parameter(
            'mask_sync_tolerance_s',
            0.12
        )

        self.scenario = str(
            self.get_parameter('scenario').value
        )

        self.object_name = str(
            self.get_parameter('object_name').value
        )

        self.condition = str(
            self.get_parameter('condition').value
        )

        self.position_filter = str(
            self.get_parameter(
                'position_filter'
            ).value
        )

        self.positions_file = os.path.expanduser(
            str(
                self.get_parameter(
                    'positions_file'
                ).value
            )
        )

        self.ground_truth_file = os.path.expanduser(
            str(
                self.get_parameter(
                    'ground_truth_file'
                ).value
            )
        )

        self.robot_entity = str(
            self.get_parameter('robot_entity').value
        )

        self.detection_timeout_s = float(
            self.get_parameter(
                'detection_timeout_s'
            ).value
        )

        self.navigation_timeout_s = float(
            self.get_parameter(
                'navigation_timeout_s'
            ).value
        )

        self.navigation_wall_watchdog_s = float(
            self.get_parameter(
                'navigation_wall_watchdog_s'
            ).value
        )

        self.settle_time_s = float(
            self.get_parameter(
                'settle_time_s'
            ).value
        )

        self.goal_region_tolerance_m = float(
            self.get_parameter(
                'goal_region_tolerance_m'
            ).value
        )

        self.mask_sync_tolerance_s = float(
            self.get_parameter(
                'mask_sync_tolerance_s'
            ).value
        )

        # Las cuatro posiciones experimentales.
        self.position_ids = [
            'P1',
            'P2',
            'P3',
            'P4',
        ]

        if self.position_filter != 'ALL':

            if self.position_filter not in self.position_ids:
                raise RuntimeError(
                    'position_filter debe ser '
                    'ALL, P1, P2, P3 o P4'
                )

            self.position_ids = [
                self.position_filter
            ]

        (
            self.positions,
            self.target_reference,
            self.target_region,
        ) = self.load_positions_file()

        self.ground_truth = None

        if self.ground_truth_file:
            self.ground_truth = (
                self.load_ground_truth_file()
            )

        if (
            self.ground_truth is None
            and (
                self.target_reference is None
                or self.target_region is None
            )
        ):
            raise RuntimeError(
                'Debe existir target_region legacy '
                'o ground_truth_file'
            )

        # ---------------------------------------------------------
        # Estado interno
        # ---------------------------------------------------------

        self.current_position_id = None
        self.latest_goal = None
        self.latest_target_point = None

        # TF usado únicamente por el evaluador para relacionar
        # odom con el frame físico Isaac World.
        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # Transformación World <- odom congelada al inicio
        # de cada episodio.
        self.episode_world_odom = None

        self.navigation_started = False
        self.navigation_finished = False
        self.navigation_status = None

        # Métricas geométricas de navegación.
        self.navigation_distance_m = 0.0
        self.navigation_previous_xy = None
        self.navigation_final_xy = None

        # Permite distinguir el goal Nav2 actual de goals anteriores.
        self.latest_action_goal_stamp_ns = 0
        self.action_goal_baseline_ns = 0

        # ---------------------------------------------------------
        # Estado de evaluación de máscaras
        # ---------------------------------------------------------

        # JSON más reciente publicado por Isaac:
        # ID de segmentación -> prim de Isaac.
        self.semantic_labels = {}

        # Se guarda la máscara GT comprimida con packbits.
        # 300 frames ocupan mucha menos memoria que conservar
        # las imágenes 32SC1 completas.
        self.gt_mask_buffer = deque(
            maxlen=300
        )

        # Resultados IoU calculados a partir de selected_mask.
        self.mask_metric_buffer = deque(
            maxlen=50
        )

        # Métrica congelada para el episodio actual.
        self.episode_mask_metrics = None

        # ---------------------------------------------------------
        # Suscripciones
        # ---------------------------------------------------------

        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/semantic_object/goal_pose',
            self.goal_callback,
            10
        )

        self.target_point_subscription = self.create_subscription(
            PointStamped,
            '/semantic_object/point_odom_stable',
            self.target_point_callback,
            10
        )

        self.status_subscription = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_callback,
            10
        )

        # Máscara de la instancia seleccionada por nuestro
        # semantic_localizer.
        self.selected_mask_subscription = (
            self.create_subscription(
                Image,
                '/semantic_object/selected_mask',
                self.selected_mask_callback,
                qos_profile_sensor_data
            )
        )

        # Ground truth visual de Isaac Sim.
        # BEST_EFFORT como subscriber es compatible con el
        # publisher RELIABLE de Isaac.
        self.instance_segmentation_subscription = (
            self.create_subscription(
                Image,
                '/instance_segmentation',
                self.instance_segmentation_callback,
                qos_profile_sensor_data
            )
        )

        # Correspondencia dinámica:
        # instance ID -> prim de Isaac.
        self.semantic_labels_subscription = (
            self.create_subscription(
                String,
                '/semantic_labels',
                self.semantic_labels_callback,
                10
            )
        )

        # ---------------------------------------------------------
        # Clientes de servicios
        # ---------------------------------------------------------

        self.simulation_state_client = self.create_client(
            SetSimulationState,
            '/set_simulation_state'
        )

        self.get_entity_state_client = self.create_client(
            GetEntityState,
            '/get_entity_state'
        )

        self.set_entity_state_client = self.create_client(
            SetEntityState,
            '/set_entity_state'
        )

        self.local_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )

        self.global_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )

        self.reset_target_client = self.create_client(
            Trigger,
            '/semantic_object/reset_target'
        )

        self.navigator_parameter_client = self.create_client(
            SetParameters,
            '/semantic_navigator/set_parameters'
        )

        self.cancel_navigation_client = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal'
        )

        # ---------------------------------------------------------
        # Resultados
        # ---------------------------------------------------------

        output_directory = os.path.expanduser(
            '~/indoor/evaluation/automated_runs'
        )

        os.makedirs(
            output_directory,
            exist_ok=True
        )

        timestamp = datetime.now().strftime(
            '%Y%m%d_%H%M%S'
        )

        self.csv_path = os.path.join(
            output_directory,
            f'{self.scenario}_{timestamp}.csv'
        )

        with open(
            self.csv_path,
            'w',
            newline='',
            encoding='utf-8'
        ) as csv_file:

            writer = csv.writer(csv_file)

            writer.writerow([
                'episode_id',
                'position_id',
                'scenario',
                'object',
                'condition',
                'start_x',
                'start_y',
                'detection_success',
                'target_x',
                'target_y',
                'target_z',
                'target_error_m',
                'target_correct',
                'target_world_x',
                'target_world_y',
                'target_world_z',
                'localization_error_3d_center_m',
                'localization_error_3d_bbox_m',
                'mask_iou',
                'mask_sync_dt_s',
                'mask_pred_area_px',
                'mask_gt_area_px',
                'mask_intersection_px',
                'mask_union_px',
                'mask_gt_instance',
                'navigation_status',
                'navigation_time_sim_s',
                'goal_x',
                'goal_y',
                'navigation_time_real_s',
                'distance_traveled_m',
                'final_robot_x',
                'final_robot_y',
                'final_goal_error_m',
                'goal_region_reached',
                'goal_region_tolerance_m',
                'navigation_timeout',
                'failure_stage',
                'failure_reason',
            ])

        self.get_logger().info(
            'Evaluation Runner iniciado'
        )

        self.get_logger().info(
            f'Escenario: {self.scenario}'
        )

        self.get_logger().info(
            'Posiciones: '
            + ', '.join(self.position_ids)
        )

        self.get_logger().info(
            f'Condición: {self.condition}'
        )

        self.get_logger().info(
            f'Archivo de poses: {self.positions_file}'
        )

        self.get_logger().info(
            f'Resultados: {self.csv_path}'
        )

    # =============================================================
    # CALLBACKS
    # =============================================================

    def goal_callback(self, message):
        self.latest_goal = message

    def target_point_callback(self, message):
        self.latest_target_point = message

    def stamp_to_seconds(self, stamp):

        return (
            float(stamp.sec)
            + float(stamp.nanosec) / 1e9
        )


    def semantic_labels_callback(
        self,
        message
    ):

        try:
            labels = json.loads(
                message.data
            )

            if isinstance(labels, dict):
                self.semantic_labels = labels

        except Exception as error:
            self.get_logger().warning(
                'No se pudo interpretar '
                f'/semantic_labels: {error}'
            )


    def extract_label_strings(
        self,
        value
    ):
        """
        Extrae strings incluso si Isaac cambia alguna entrada
        del JSON a dict/list.
        """

        if isinstance(value, str):
            return [value]

        if isinstance(value, dict):

            strings = []

            for nested_value in value.values():
                strings.extend(
                    self.extract_label_strings(
                        nested_value
                    )
                )

            return strings

        if isinstance(value, (list, tuple)):

            strings = []

            for nested_value in value:
                strings.extend(
                    self.extract_label_strings(
                        nested_value
                    )
                )

            return strings

        return []


    def get_expected_gt_ids(
        self,
        position_id
    ):
        """
        Resuelve dinámicamente todos los IDs de segmentación
        que pertenecen a la instancia GT esperada.

        Nunca se fijan IDs como 494 o 499 en el código.
        """

        if self.ground_truth is None:
            return None

        if not self.semantic_labels:
            return None

        expected_config = (
            self.ground_truth[
                'expected'
            ].get(position_id)
        )

        if expected_config is None:
            return None

        instance_name = expected_config[
            'instance'
        ]

        instance_config = (
            self.ground_truth[
                'instances'
            ][instance_name]
        )

        entity = instance_config.get(
            'entity'
        )

        if not entity:
            return None

        ids = []

        for id_text, value in (
            self.semantic_labels.items()
        ):

            paths = self.extract_label_strings(
                value
            )

            belongs_to_instance = any(
                path == entity
                or path.startswith(
                    entity + '/'
                )
                for path in paths
            )

            if not belongs_to_instance:
                continue

            try:
                ids.append(
                    int(id_text)
                )
            except ValueError:
                continue

        if not ids:
            return None

        return {
            'instance': instance_name,
            'entity': entity,
            'ids': sorted(set(ids)),
        }


    def decode_instance_segmentation(
        self,
        message
    ):

        if message.encoding != '32SC1':
            return None

        if (
            message.height <= 0
            or message.width <= 0
            or message.step <= 0
        ):
            return None

        if message.step % 4 != 0:
            return None

        row_values = (
            message.step // 4
        )

        expected_bytes = (
            int(message.height)
            * int(message.step)
        )

        if len(message.data) < expected_bytes:
            return None

        dtype = np.dtype(
            '>i4'
            if message.is_bigendian
            else '<i4'
        )

        image = np.frombuffer(
            message.data,
            dtype=dtype,
            count=(
                int(message.height)
                * row_values
            )
        )

        image = image.reshape(
            int(message.height),
            row_values
        )

        return image[
            :,
            :int(message.width)
        ]


    def decode_mono8_mask(
        self,
        message
    ):

        if message.encoding not in (
            'mono8',
            '8UC1',
        ):
            return None

        if (
            message.height <= 0
            or message.width <= 0
            or message.step <= 0
        ):
            return None

        expected_bytes = (
            int(message.height)
            * int(message.step)
        )

        if len(message.data) < expected_bytes:
            return None

        image = np.frombuffer(
            message.data,
            dtype=np.uint8,
            count=expected_bytes
        )

        image = image.reshape(
            int(message.height),
            int(message.step)
        )

        return (
            image[
                :,
                :int(message.width)
            ]
            > 0
        )


    def instance_segmentation_callback(
        self,
        message
    ):
        """
        Convierte /instance_segmentation en una máscara
        binaria solamente de la instancia GT esperada para
        P1/P2/P3/P4 y la guarda comprimida.
        """

        if (
            self.ground_truth is None
            or self.current_position_id is None
        ):
            return

        metadata = self.get_expected_gt_ids(
            self.current_position_id
        )

        if metadata is None:
            return

        instance_image = (
            self.decode_instance_segmentation(
                message
            )
        )

        if instance_image is None:
            return

        gt_mask = np.isin(
            instance_image,
            metadata['ids']
        )

        gt_area = int(
            np.count_nonzero(
                gt_mask
            )
        )

        packed_mask = np.packbits(
            gt_mask.reshape(-1)
        )

        self.gt_mask_buffer.append({
            'stamp': self.stamp_to_seconds(
                message.header.stamp
            ),
            'position_id': (
                self.current_position_id
            ),
            'height': int(
                message.height
            ),
            'width': int(
                message.width
            ),
            'packed_mask': packed_mask,
            'gt_area_px': gt_area,
            'instance': metadata[
                'instance'
            ],
            'entity': metadata[
                'entity'
            ],
            'ids': metadata[
                'ids'
            ],
        })


    def selected_mask_callback(
        self,
        message
    ):
        """
        Compara la máscara de la instancia seleccionada por
        semantic_localizer con la máscara GT de Isaac cuyo
        timestamp sea más cercano.
        """

        if (
            self.ground_truth is None
            or self.current_position_id is None
            or not self.gt_mask_buffer
        ):
            return

        pred_mask = self.decode_mono8_mask(
            message
        )

        if pred_mask is None:
            return

        pred_stamp = self.stamp_to_seconds(
            message.header.stamp
        )

        candidates = [
            item
            for item in self.gt_mask_buffer
            if (
                item['position_id']
                == self.current_position_id
                and item['height']
                == pred_mask.shape[0]
                and item['width']
                == pred_mask.shape[1]
            )
        ]

        if not candidates:
            return

        nearest = min(
            candidates,
            key=lambda item: abs(
                item['stamp']
                - pred_stamp
            )
        )

        sync_dt = abs(
            nearest['stamp']
            - pred_stamp
        )

        if (
            sync_dt
            > self.mask_sync_tolerance_s
        ):
            return

        pixel_count = (
            int(nearest['height'])
            * int(nearest['width'])
        )

        gt_mask = np.unpackbits(
            nearest['packed_mask'],
            count=pixel_count
        ).reshape(
            nearest['height'],
            nearest['width']
        ).astype(bool)

        pred_area = int(
            np.count_nonzero(
                pred_mask
            )
        )

        intersection = int(
            np.count_nonzero(
                pred_mask
                & gt_mask
            )
        )

        union = int(
            np.count_nonzero(
                pred_mask
                | gt_mask
            )
        )

        if union <= 0:
            return

        iou = (
            float(intersection)
            / float(union)
        )

        self.mask_metric_buffer.append({
            'stamp': pred_stamp,
            'position_id': (
                self.current_position_id
            ),
            'mask_iou': iou,
            'mask_sync_dt_s': sync_dt,
            'mask_pred_area_px': pred_area,
            'mask_gt_area_px': nearest[
                'gt_area_px'
            ],
            'mask_intersection_px': (
                intersection
            ),
            'mask_union_px': union,
            'mask_gt_instance': nearest[
                'instance'
            ],
            'mask_gt_ids': nearest[
                'ids'
            ],
        })


    def freeze_episode_mask_metrics(
        self
    ):
        """
        Se llama después de obtener el primer target estable y
        antes de habilitar Nav2.

        Por ello el IoU pertenece a la fase de percepción y no
        a imágenes tomadas mientras el robot ya navega.
        """

        self.episode_mask_metrics = None

        if self.ground_truth is None:
            return

        # Normalmente ya existe una métrica. Damos hasta 1 s
        # real para permitir que DDS entregue callbacks que
        # pertenezcan a la detección recién estabilizada.
        deadline = (
            time.monotonic()
            + 1.0
        )

        while (
            rclpy.ok()
            and not self.mask_metric_buffer
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

        if not self.mask_metric_buffer:

            self.get_logger().warning(
                'No se pudo obtener IoU válido '
                'para este episodio.'
            )

            return

        # Tomamos la última máscara válida disponible antes
        # de iniciar la navegación.
        self.episode_mask_metrics = dict(
            self.mask_metric_buffer[-1]
        )

        metric = self.episode_mask_metrics

        self.get_logger().info(
            'IoU de instancia congelado | '
            f'GT={metric["mask_gt_instance"]} | '
            f'IoU={metric["mask_iou"]:.4f} | '
            f'dt={metric["mask_sync_dt_s"]:.4f} s | '
            f'pred={metric["mask_pred_area_px"]} px | '
            f'GT={metric["mask_gt_area_px"]} px'
        )


    def status_callback(self, message):

        if not message.status_list:
            return

        # Elegimos el goal creado más recientemente.
        newest_status = max(
            message.status_list,
            key=lambda item: (
                item.goal_info.stamp.sec * 1_000_000_000
                + item.goal_info.stamp.nanosec
            )
        )

        stamp_ns = (
            newest_status.goal_info.stamp.sec * 1_000_000_000
            + newest_status.goal_info.stamp.nanosec
        )

        self.latest_action_goal_stamp_ns = max(
            self.latest_action_goal_stamp_ns,
            stamp_ns
        )

        # Ignoramos goals pertenecientes a episodios anteriores.
        if stamp_ns <= self.action_goal_baseline_ns:
            return

        status = newest_status.status

        if status in (1, 2, 3):
            self.navigation_started = True

        if status in (4, 5, 6):
            self.navigation_status = status
            self.navigation_finished = True

    # =============================================================
    # UTILIDADES ROS
    # =============================================================

    def wait_for_service(
        self,
        client,
        name,
        timeout=20.0
    ):

        self.get_logger().info(
            f'Esperando servicio {name}...'
        )

        if not client.wait_for_service(
            timeout_sec=timeout
        ):
            raise RuntimeError(
                f'Servicio no disponible: {name}'
            )

    def call_service(
        self,
        client,
        request,
        timeout=20.0
    ):

        future = client.call_async(request)

        start = time.monotonic()

        while (
            rclpy.ok()
            and not future.done()
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if (
                time.monotonic() - start
                > timeout
            ):
                raise RuntimeError(
                    'Timeout esperando servicio'
                )

        return future.result()

    # =============================================================
    # SEMANTIC NAVIGATOR
    # =============================================================

    def set_navigator_enabled(
        self,
        enabled
    ):

        request = SetParameters.Request()

        parameter = Parameter()

        parameter.name = 'enabled'

        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_BOOL,
            bool_value=enabled
        )

        request.parameters = [
            parameter
        ]

        response = self.call_service(
            self.navigator_parameter_client,
            request
        )

        if (
            not response.results
            or not response.results[0].successful
        ):
            raise RuntimeError(
                'No se pudo cambiar enabled '
                'en semantic_navigator'
            )

        self.get_logger().info(
            f'semantic_navigator enabled={enabled}'
        )

    # =============================================================
    # POSICIONES EXPERIMENTALES
    # =============================================================

    def load_positions_file(self):

        if not os.path.isfile(
            self.positions_file
        ):
            raise RuntimeError(
                'No existe el archivo de posiciones: '
                f'{self.positions_file}'
            )

        with open(
            self.positions_file,
            'r',
            encoding='utf-8'
        ) as yaml_file:

            data = yaml.safe_load(
                yaml_file
            ) or {}

        config_object = data.get(
            'object'
        )

        if (
            config_object is not None
            and config_object != self.object_name
        ):
            raise RuntimeError(
                'El archivo de posiciones pertenece a '
                f'{config_object}, pero object_name='
                f'{self.object_name}'
            )

        positions = data.get(
            'positions',
            {}
        )

        required_fields = [
            'x',
            'y',
            'z',
            'qx',
            'qy',
            'qz',
            'qw',
        ]

        for position_id in self.position_ids:

            if position_id not in positions:
                raise RuntimeError(
                    f'Falta {position_id} '
                    'en el archivo de posiciones'
                )

            missing_fields = [
                field
                for field in required_fields
                if field not in positions[position_id]
            ]

            if missing_fields:
                raise RuntimeError(
                    f'{position_id} incompleta. '
                    f'Faltan: {missing_fields}'
                )

        target_reference = data.get(
            'target_reference'
        )

        target_region = data.get(
            'target_region'
        )

        # Modo legacy: usado por boxes.
        # En modo multi-instancia estos dos campos pueden
        # estar ausentes porque el GT viene de otro YAML.
        if (
            target_reference is None
            and target_region is None
        ):
            target_reference = None
            target_region = None

        elif (
            not isinstance(target_reference, dict)
            or not isinstance(target_region, dict)
        ):
            raise RuntimeError(
                'target_reference y target_region '
                'deben existir juntos'
            )

        else:
            required_target_fields = [
                'x',
                'y',
                'z',
                'tolerance_m',
            ]

            missing_target_fields = [
                field
                for field in required_target_fields
                if field not in target_reference
            ]

            if missing_target_fields:
                raise RuntimeError(
                    'target_reference incompleto. '
                    f'Faltan: {missing_target_fields}'
                )

            target_reference = {
                field: float(
                    target_reference[field]
                )
                for field in required_target_fields
            }

            if (
                target_reference[
                    'tolerance_m'
                ]
                <= 0.0
            ):
                raise RuntimeError(
                    'target_reference.tolerance_m '
                    'debe ser mayor que cero'
                )

            required_region_fields = [
                'xmin',
                'xmax',
                'ymin',
                'ymax',
            ]

            missing_region_fields = [
                field
                for field in required_region_fields
                if field not in target_region
            ]

            if missing_region_fields:
                raise RuntimeError(
                    'target_region incompleto. '
                    f'Faltan: {missing_region_fields}'
                )

            target_region = {
                field: float(
                    target_region[field]
                )
                for field in required_region_fields
            }

            if (
                target_region['xmin']
                >= target_region['xmax']
                or
                target_region['ymin']
                >= target_region['ymax']
            ):
                raise RuntimeError(
                    'target_region contiene '
                    'límites inválidos'
                )

        return (
            positions,
            target_reference,
            target_region,
        )


    # =============================================================
    # GROUND TRUTH MULTI-INSTANCIA
    # =============================================================

    def load_ground_truth_file(self):

        if not os.path.isfile(
            self.ground_truth_file
        ):
            raise RuntimeError(
                'No existe ground_truth_file: '
                f'{self.ground_truth_file}'
            )

        with open(
            self.ground_truth_file,
            'r',
            encoding='utf-8'
        ) as yaml_file:

            data = yaml.safe_load(
                yaml_file
            ) or {}

        config_object = data.get(
            'object'
        )

        if (
            config_object is not None
            and config_object
            != self.object_name
        ):
            raise RuntimeError(
                'El ground truth pertenece a '
                f'{config_object}, pero object_name='
                f'{self.object_name}'
            )

        instances = data.get(
            'instances'
        )

        expected = data.get(
            'expected_nearest_instance'
        )

        if not isinstance(
            instances,
            dict
        ):
            raise RuntimeError(
                'Falta instances en ground truth'
            )

        if not isinstance(
            expected,
            dict
        ):
            raise RuntimeError(
                'Falta expected_nearest_instance '
                'en ground truth'
            )

        margin = float(
            data.get(
                'target_margin_m',
                0.20
            )
        )

        if margin < 0.0:
            raise RuntimeError(
                'target_margin_m no puede ser negativo'
            )

        for instance_name, config in (
            instances.items()
        ):

            entity = config.get(
                'entity'
            )

            if (
                not isinstance(entity, str)
                or not entity
            ):
                raise RuntimeError(
                    f'Falta entity de '
                    f'{instance_name}'
                )

            bounds = config.get(
                'bounds'
            )

            center = config.get(
                'center'
            )

            if not isinstance(
                bounds,
                dict
            ):
                raise RuntimeError(
                    f'Faltan bounds de '
                    f'{instance_name}'
                )

            for field in [
                'xmin',
                'xmax',
                'ymin',
                'ymax',
                'zmin',
                'zmax',
            ]:
                if field not in bounds:
                    raise RuntimeError(
                        f'Falta {field} en '
                        f'{instance_name}'
                    )

            if not isinstance(
                center,
                dict
            ):
                raise RuntimeError(
                    f'Falta center de '
                    f'{instance_name}'
                )

            if (
                'x' not in center
                or 'y' not in center
                or 'z' not in center
            ):
                raise RuntimeError(
                    f'Center incompleto de '
                    f'{instance_name}'
                )

        for position_id in self.position_ids:

            if position_id not in expected:
                raise RuntimeError(
                    'Falta instancia esperada para '
                    f'{position_id}'
                )

            instance_name = expected[
                position_id
            ].get(
                'instance'
            )

            if instance_name not in instances:
                raise RuntimeError(
                    f'{position_id} referencia '
                    f'instancia desconocida: '
                    f'{instance_name}'
                )

        self.get_logger().info(
            'Ground truth multi-instancia cargado | '
            f'instancias={list(instances.keys())} | '
            f'margen={margin:.2f} m'
        )

        return {
            'instances': instances,
            'expected': expected,
            'margin': margin,
        }


    def quaternion_to_yaw(
        self,
        qx,
        qy,
        qz,
        qw
    ):

        siny_cosp = 2.0 * (
            qw * qz
            + qx * qy
        )

        cosy_cosp = 1.0 - 2.0 * (
            qy * qy
            + qz * qz
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )


    def capture_world_odom_transform(
        self,
        position_id
    ):
        """
        Calcula y congela:

            Isaac World <- odom

        utilizando la pose conocida del robot en World y un
        conjunto de muestras TF odom -> base_link tomadas después
        de la estabilización.

        No se acepta inmediatamente el último TF disponible:
        se exige que sea reciente, que existan varias muestras
        distintas y que la pose sea estable.
        """

        pose = self.positions[
            position_id
        ]

        world_base_x = float(
            pose['x']
        )

        world_base_y = float(
            pose['y']
        )

        world_base_z = float(
            pose['z']
        )

        world_base_yaw = (
            self.quaternion_to_yaw(
                float(pose['qx']),
                float(pose['qy']),
                float(pose['qz']),
                float(pose['qw']),
            )
        )

        # -----------------------------------------------------
        # Captura robusta de TF
        # -----------------------------------------------------

        required_samples = 5

        # El TF debe ser suficientemente reciente respecto
        # a /clock de Isaac.
        max_tf_age_s = 0.50

        # Separación mínima temporal entre muestras aceptadas.
        sample_interval_sim_s = 0.10

        # Durante el settle el robot debería estar quieto.
        max_xy_spread_m = 0.05
        max_yaw_spread_rad = math.radians(
            2.0
        )

        # Watchdog únicamente de tiempo real.
        wall_timeout_s = 30.0

        samples = []

        wall_start = time.monotonic()

        last_sample_sim_ns = None
        last_tf_stamp_ns = None
        last_error = None
        last_tf_age_s = None

        self.get_logger().info(
            'Esperando TF estable odom -> base_link...'
        )

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

            now_sim_ns = (
                self.get_clock().now().nanoseconds
            )

            if (
                time.monotonic() - wall_start
                >= wall_timeout_s
            ):
                raise RuntimeError(
                    'No se pudo obtener un TF estable '
                    'odom -> base_link después de '
                    f'{wall_timeout_s:.1f} s reales. '
                    f'Muestras válidas={len(samples)}. '
                    f'Último age={last_tf_age_s}. '
                    f'Último error={last_error}'
                )

            try:

                transform = (
                    self.tf_buffer.lookup_transform(
                        'odom',
                        'base_link',
                        Time(),
                        timeout=Duration(
                            seconds=0.20
                        )
                    )
                )

            except Exception as error:

                last_error = str(error)
                continue

            stamp = transform.header.stamp

            tf_stamp_ns = (
                int(stamp.sec)
                * 1_000_000_000
                + int(stamp.nanosec)
            )

            # Ignorar TF sin timestamp válido.
            if tf_stamp_ns <= 0:
                continue

            tf_age_s = (
                now_sim_ns
                - tf_stamp_ns
            ) / 1_000_000_000.0

            last_tf_age_s = tf_age_s

            # No aceptar transformaciones viejas anteriores
            # al estado actual de la simulación.
            if (
                tf_age_s < -0.05
                or tf_age_s > max_tf_age_s
            ):
                continue

            # No contar varias veces exactamente el mismo TF.
            if (
                last_tf_stamp_ns is not None
                and tf_stamp_ns
                <= last_tf_stamp_ns
            ):
                continue

            # Separar las muestras en tiempo simulado.
            if (
                last_sample_sim_ns is not None
                and (
                    now_sim_ns
                    - last_sample_sim_ns
                )
                < int(
                    sample_interval_sim_s
                    * 1_000_000_000
                )
            ):
                continue

            translation = (
                transform.transform.translation
            )

            rotation = (
                transform.transform.rotation
            )

            sample_yaw = (
                self.quaternion_to_yaw(
                    float(rotation.x),
                    float(rotation.y),
                    float(rotation.z),
                    float(rotation.w),
                )
            )

            samples.append(
                {
                    'x': float(
                        translation.x
                    ),
                    'y': float(
                        translation.y
                    ),
                    'z': float(
                        translation.z
                    ),
                    'yaw': float(
                        sample_yaw
                    ),
                    'stamp_ns':
                        tf_stamp_ns,
                }
            )

            last_tf_stamp_ns = (
                tf_stamp_ns
            )

            last_sample_sim_ns = (
                now_sim_ns
            )

            # Ventana móvil.
            if (
                len(samples)
                > required_samples
            ):
                samples.pop(0)

            if (
                len(samples)
                < required_samples
            ):
                continue

            xs = np.array(
                [
                    sample['x']
                    for sample in samples
                ],
                dtype=np.float64
            )

            ys = np.array(
                [
                    sample['y']
                    for sample in samples
                ],
                dtype=np.float64
            )

            zs = np.array(
                [
                    sample['z']
                    for sample in samples
                ],
                dtype=np.float64
            )

            yaws = np.array(
                [
                    sample['yaw']
                    for sample in samples
                ],
                dtype=np.float64
            )

            odom_base_x = float(
                np.median(xs)
            )

            odom_base_y = float(
                np.median(ys)
            )

            odom_base_z = float(
                np.median(zs)
            )

            # Unwrap evita problemas cerca de -pi/+pi.
            unwrapped_yaws = np.unwrap(
                yaws
            )

            odom_base_yaw = float(
                np.median(
                    unwrapped_yaws
                )
            )

            # Normalizamos nuevamente.
            odom_base_yaw = math.atan2(
                math.sin(
                    odom_base_yaw
                ),
                math.cos(
                    odom_base_yaw
                )
            )

            xy_spread = float(
                np.max(
                    np.hypot(
                        xs - odom_base_x,
                        ys - odom_base_y
                    )
                )
            )

            yaw_errors = np.array(
                [
                    math.atan2(
                        math.sin(
                            yaw - odom_base_yaw
                        ),
                        math.cos(
                            yaw - odom_base_yaw
                        )
                    )
                    for yaw in yaws
                ],
                dtype=np.float64
            )

            yaw_spread = float(
                np.max(
                    np.abs(
                        yaw_errors
                    )
                )
            )

            if (
                xy_spread
                > max_xy_spread_m
                or
                yaw_spread
                > max_yaw_spread_rad
            ):

                self.get_logger().warning(
                    'TF todavía inestable | '
                    f'XY spread={xy_spread:.3f} m | '
                    f'yaw spread='
                    f'{math.degrees(yaw_spread):.2f} deg'
                )

                # Conservamos la ventana móvil para que
                # nuevas muestras desplacen a las antiguas.
                continue

            self.get_logger().info(
                'TF estable confirmado | '
                f'muestras={required_samples} | '
                f'XY spread={xy_spread:.3f} m | '
                f'yaw spread='
                f'{math.degrees(yaw_spread):.2f} deg | '
                f'age={tf_age_s:.3f} s'
            )

            break

        # -----------------------------------------------------
        # World <- odom
        # -----------------------------------------------------

        world_odom_yaw = (
            world_base_yaw
            - odom_base_yaw
        )

        world_odom_yaw = math.atan2(
            math.sin(
                world_odom_yaw
            ),
            math.cos(
                world_odom_yaw
            )
        )

        c = math.cos(
            world_odom_yaw
        )

        s = math.sin(
            world_odom_yaw
        )

        world_odom_x = (
            world_base_x
            - (
                c * odom_base_x
                - s * odom_base_y
            )
        )

        world_odom_y = (
            world_base_y
            - (
                s * odom_base_x
                + c * odom_base_y
            )
        )

        world_odom_z = (
            world_base_z
            - odom_base_z
        )

        self.episode_world_odom = {
            'x': world_odom_x,
            'y': world_odom_y,
            'z': world_odom_z,
            'yaw': world_odom_yaw,
        }

        self.get_logger().info(
            'Transformación GT congelada | '
            f'{position_id} | '
            f'base_world=('
            f'{world_base_x:.3f}, '
            f'{world_base_y:.3f}, '
            f'{math.degrees(world_base_yaw):.2f} deg) | '
            f'base_odom=('
            f'{odom_base_x:.3f}, '
            f'{odom_base_y:.3f}, '
            f'{math.degrees(odom_base_yaw):.2f} deg) | '
            f'world<-odom=('
            f'{world_odom_x:.3f}, '
            f'{world_odom_y:.3f}, '
            f'{math.degrees(world_odom_yaw):.2f} deg)'
        )


    def target_odom_to_world(
        self,
        position_id,
        target_x,
        target_y,
        target_z
    ):
        """
        Convierte target XY desde odom a Isaac World usando
        la transformación congelada al inicio del episodio.
        """

        if self.episode_world_odom is None:
            raise RuntimeError(
                'No existe transformación '
                'World <- odom para este episodio'
            )

        tx = float(
            self.episode_world_odom['x']
        )

        ty = float(
            self.episode_world_odom['y']
        )

        tz = float(
            self.episode_world_odom['z']
        )

        yaw = float(
            self.episode_world_odom['yaw']
        )

        c = math.cos(
            yaw
        )

        s = math.sin(
            yaw
        )

        world_x = (
            tx
            + c * target_x
            - s * target_y
        )

        world_y = (
            ty
            + s * target_x
            + c * target_y
        )

        world_z = (
            tz
            + target_z
        )

        return (
            world_x,
            world_y,
            world_z,
        )


    def evaluate_multi_instance_target(
        self,
        position_id,
        target_x,
        target_y,
        target_z
    ):

        world_x, world_y, world_z = (
            self.target_odom_to_world(
                position_id,
                target_x,
                target_y,
                target_z
            )
        )

        expected_config = (
            self.ground_truth[
                'expected'
            ][position_id]
        )

        expected_instance = (
            expected_config[
                'instance'
            ]
        )

        instance_config = (
            self.ground_truth[
                'instances'
            ][expected_instance]
        )

        bounds = instance_config[
            'bounds'
        ]

        margin = self.ground_truth[
            'margin'
        ]

        xmin = float(
            bounds['xmin']
        ) - margin

        xmax = float(
            bounds['xmax']
        ) + margin

        ymin = float(
            bounds['ymin']
        ) - margin

        ymax = float(
            bounds['ymax']
        ) + margin

        correct = int(
            xmin
            <= world_x
            <= xmax
            and
            ymin
            <= world_y
            <= ymax
        )

        center = instance_config[
            'center'
        ]

        # Error 3D respecto al centro geométrico real.
        dx_center = (
            world_x
            - float(center['x'])
        )

        dy_center = (
            world_y
            - float(center['y'])
        )

        dz_center = (
            world_z
            - float(center['z'])
        )

        error_3d_center = math.sqrt(
            dx_center * dx_center
            + dy_center * dy_center
            + dz_center * dz_center
        )

        # Distancia 3D mínima al bounding box físico real.
        # Si el punto está dentro del volumen, el error es 0.
        raw_xmin = float(bounds['xmin'])
        raw_xmax = float(bounds['xmax'])
        raw_ymin = float(bounds['ymin'])
        raw_ymax = float(bounds['ymax'])
        raw_zmin = float(bounds['zmin'])
        raw_zmax = float(bounds['zmax'])

        dx_bbox = max(
            raw_xmin - world_x,
            0.0,
            world_x - raw_xmax
        )

        dy_bbox = max(
            raw_ymin - world_y,
            0.0,
            world_y - raw_ymax
        )

        dz_bbox = max(
            raw_zmin - world_z,
            0.0,
            world_z - raw_zmax
        )

        error_3d_bbox = math.sqrt(
            dx_bbox * dx_bbox
            + dy_bbox * dy_bbox
            + dz_bbox * dz_bbox
        )

        return (
            correct,
            error_3d_center,
            error_3d_bbox,
            world_x,
            world_y,
            world_z,
            expected_instance,
            xmin,
            xmax,
            ymin,
            ymax,
        )


    # =============================================================
    # SIMULACIÓN
    # =============================================================

    def set_simulation_state(
        self,
        state
    ):

        request = SetSimulationState.Request()

        request.state.state = state

        response = self.call_service(
            self.simulation_state_client,
            request
        )

        if response.result.result not in (
            1,
            101,
        ):
            raise RuntimeError(
                'No se pudo cambiar estado '
                f'de simulación: '
                f'{response.result.error_message}'
            )

    def restore_robot(
        self,
        position_id
    ):

        if position_id not in self.positions:
            raise RuntimeError(
                f'Posición desconocida: {position_id}'
            )

        pose_config = self.positions[
            position_id
        ]

        request = SetEntityState.Request()

        request.entity = self.robot_entity

        state = EntityState()

        state.header.frame_id = 'world'

        state.pose.position.x = float(
            pose_config['x']
        )

        state.pose.position.y = float(
            pose_config['y']
        )

        state.pose.position.z = float(
            pose_config['z']
        )

        state.pose.orientation.x = float(
            pose_config['qx']
        )

        state.pose.orientation.y = float(
            pose_config['qy']
        )

        state.pose.orientation.z = float(
            pose_config['qz']
        )

        state.pose.orientation.w = float(
            pose_config['qw']
        )

        # Cada episodio empieza completamente detenido.
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
            request
        )

        if response.result.result != 1:
            raise RuntimeError(
                'No se pudo colocar el robot '
                f'en {position_id}'
            )

        self.current_position_id = (
            position_id
        )

        self.get_logger().info(
            f'Robot colocado en {position_id} | '
            f'x={pose_config["x"]:.3f}, '
            f'y={pose_config["y"]:.3f}'
        )


    # =============================================================
    # RESET DEL EPISODIO
    # =============================================================

    def clear_costmaps(self):

        request = ClearEntireCostmap.Request()

        self.call_service(
            self.local_costmap_client,
            request
        )

        self.call_service(
            self.global_costmap_client,
            request
        )

    def reset_semantic_target(self):

        request = Trigger.Request()

        response = self.call_service(
            self.reset_target_client,
            request
        )

        if not response.success:
            raise RuntimeError(
                'No se pudo resetear '
                'semantic_localizer'
            )

    def cancel_active_navigation(self):

        self.get_logger().warning(
            'Cancelando navegación activa de Nav2...'
        )

        request = CancelGoal.Request()

        response = self.call_service(
            self.cancel_navigation_client,
            request,
            timeout=10.0
        )

        if response is None:
            self.get_logger().warning(
                'No se recibió respuesta de Nav2'
            )
            return

        self.get_logger().info(
            'Respuesta de cancelación | '
            f'code={response.return_code}, '
            f'goals={len(response.goals_canceling)}'
        )

        # Permitimos que Nav2 procese la cancelación.
        end_time = time.monotonic() + 3.0

        while (
            rclpy.ok()
            and time.monotonic() < end_time
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

    def prepare_episode(
        self,
        position_id
    ):

        # Nunca usamos STOP ni reset_simulation.
        self.set_navigator_enabled(False)

        if (
            self.navigation_started
            and not self.navigation_finished
        ):
            self.cancel_active_navigation()

        # PAUSED = 2
        self.set_simulation_state(2)

        self.restore_robot(
            position_id
        )

        self.clear_costmaps()

        # No reseteamos todavía el objetivo semántico.
        # Durante los primeros segundos posteriores al teleport
        # RGB/depth/TF/CLIPSeg pueden contener frames transitorios.
        # El reset se realizará después de la estabilización.

        # Borra referencias del episodio anterior.
        self.latest_goal = None
        self.latest_target_point = None
        self.episode_world_odom = None
        self.navigation_started = False
        self.navigation_finished = False
        self.navigation_status = None

        # El IoU debe usar solamente imágenes pertenecientes
        # al episodio actual.
        self.gt_mask_buffer.clear()
        self.mask_metric_buffer.clear()
        self.episode_mask_metrics = None

        # PLAYING = 1
        self.set_simulation_state(1)

        self.get_logger().info(
            'Esperando estabilización...'
        )

        # La estabilización se mide con /clock de Isaac.
        settle_start_sim_ns = (
            self.get_clock().now().nanoseconds
        )

        settle_start_wall = time.monotonic()

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            settle_now_sim_ns = (
                self.get_clock().now().nanoseconds
            )

            settle_elapsed_sim_s = (
                settle_now_sim_ns
                - settle_start_sim_ns
            ) / 1_000_000_000.0

            if (
                settle_elapsed_sim_s
                >= self.settle_time_s
            ):
                break

            # Protección por si /clock deja de avanzar.
            if (
                time.monotonic()
                - settle_start_wall
                >= self.navigation_wall_watchdog_s
            ):
                raise RuntimeError(
                    'Watchdog alcanzado durante '
                    'la estabilización. '
                    '/clock podría estar detenido.'
                )

        # El robot todavía no ha comenzado a navegar.
        # Congelamos aquí la relación entre odom e Isaac World.
        if self.ground_truth is not None:
            self.capture_world_odom_transform(
                position_id
            )

        # ---------------------------------------------------------
        # RESET SEMÁNTICO POST-ESTABILIZACIÓN
        # ---------------------------------------------------------
        #
        # El localizador pudo recibir frames transitorios durante
        # los primeros segundos posteriores al teleport.
        #
        # Eliminamos ahora cualquier objetivo/historial que pudiera
        # haberse bloqueado durante ese periodo y obligamos a que
        # la detección oficial del episodio comience con frames
        # estabilizados.
        self.reset_semantic_target()

        # También descartamos cualquier mensaje publicado durante
        # el periodo de estabilización.
        self.latest_goal = None
        self.latest_target_point = None

        # Las métricas de máscara deben corresponder únicamente
        # a la detección fresca posterior al reset.
        self.gt_mask_buffer.clear()
        self.mask_metric_buffer.clear()
        self.episode_mask_metrics = None

        self.get_logger().info(
            'Objetivo semántico reseteado después de la estabilización.'
        )


    # =============================================================
    # EPISODIO
    # =============================================================

    def wait_for_semantic_goal(self):

        # Timeout de detección medido con /clock.
        detection_start_sim_ns = (
            self.get_clock().now().nanoseconds
        )

        # Tiempo real únicamente como watchdog.
        detection_start_wall = (
            time.monotonic()
        )

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if (
                self.latest_goal is not None
                and self.latest_target_point is not None
            ):
                return True

            detection_now_sim_ns = (
                self.get_clock().now().nanoseconds
            )

            detection_elapsed_sim_s = (
                detection_now_sim_ns
                - detection_start_sim_ns
            ) / 1_000_000_000.0

            if (
                detection_elapsed_sim_s
                >= self.detection_timeout_s
            ):
                return False

            if (
                time.monotonic()
                - detection_start_wall
                >= self.navigation_wall_watchdog_s
            ):

                self.get_logger().error(
                    'Watchdog alcanzado durante '
                    'la detección. '
                    '/clock podría estar detenido.'
                )

                return False

        return False

    def wait_for_navigation_start(self):

        start = time.monotonic()

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if self.navigation_started:
                return True

            if (
                time.monotonic() - start
                > 15.0
            ):
                return False

        return False

    def get_robot_xy_odom(self):
        """
        Obtiene la posición actual de base_link en odom.
        Esta función se usa únicamente para métricas.
        """

        try:
            transform = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                Time(),
                timeout=Duration(seconds=0.5)
            )

            return (
                float(
                    transform.transform.translation.x
                ),
                float(
                    transform.transform.translation.y
                ),
            )

        except Exception:
            return None


    def reset_navigation_metrics(self):
        """
        Comienza la medición de distancia justo después
        de que Nav2 haya aceptado la navegación.
        """

        self.navigation_distance_m = 0.0

        robot_xy = self.get_robot_xy_odom()

        self.navigation_previous_xy = robot_xy
        self.navigation_final_xy = robot_xy


    def update_navigation_distance(self):
        """
        Acumula distancia XY recorrida durante navegación.

        Movimientos menores a 1 mm se ignoran para evitar
        acumular pequeñas fluctuaciones numéricas de TF.
        """

        robot_xy = self.get_robot_xy_odom()

        if robot_xy is None:
            return

        self.navigation_final_xy = robot_xy

        if self.navigation_previous_xy is None:
            self.navigation_previous_xy = robot_xy
            return

        dx = (
            robot_xy[0]
            - self.navigation_previous_xy[0]
        )

        dy = (
            robot_xy[1]
            - self.navigation_previous_xy[1]
        )

        step_distance = math.hypot(
            dx,
            dy
        )

        if step_distance >= 0.001:
            self.navigation_distance_m += (
                step_distance
            )

        self.navigation_previous_xy = robot_xy


    def wait_for_navigation_result(self):

        # Timeout experimental medido con /clock de Isaac.
        sim_start_ns = (
            self.get_clock().now().nanoseconds
        )

        # Tiempo real usado únicamente como watchdog
        # por si la simulación o /clock se congelan.
        wall_start = time.monotonic()

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            # Muestreo de trayectoria del robot.
            self.update_navigation_distance()

            if self.navigation_finished:
                return (
                    self.navigation_status,
                    None
                )

            sim_now_ns = (
                self.get_clock().now().nanoseconds
            )

            sim_elapsed_s = (
                sim_now_ns - sim_start_ns
            ) / 1_000_000_000.0

            if (
                sim_elapsed_s
                >= self.navigation_timeout_s
            ):
                return (
                    None,
                    'TIMEOUT'
                )

            wall_elapsed_s = (
                time.monotonic()
                - wall_start
            )

            if (
                wall_elapsed_s
                >= self.navigation_wall_watchdog_s
            ):

                self.get_logger().error(
                    'Watchdog real alcanzado. '
                    '/clock podría estar detenido.'
                )

                return (
                    None,
                    'WALL_WATCHDOG'
                )

        return (
            None,
            'INTERRUPTED'
        )

    def run_episode(
        self,
        position_id
    ):

        self.get_logger().info(
            '================================'
        )

        self.get_logger().info(
            f'INICIANDO {position_id}'
        )

        self.prepare_episode(
            position_id
        )

        detection_success = (
            self.wait_for_semantic_goal()
        )

        if not detection_success:

            self.get_logger().error(
                'Timeout esperando goal semántico'
            )

            self.write_result(
                position_id,
                False,
                'NO_DETECTION',
                0.0,
                None
            )

            return

        # Congelamos la calidad de segmentación antes
        # de permitir cualquier movimiento del robot.
        self.freeze_episode_mask_metrics()

        goal = self.latest_goal

        self.get_logger().info(
            'Goal semántico detectado | '
            f'x={goal.pose.position.x:.2f}, '
            f'y={goal.pose.position.y:.2f}'
        )

        # Todo goal existente hasta este instante pertenece
        # a episodios anteriores.
        self.action_goal_baseline_ns = (
            self.latest_action_goal_stamp_ns
        )

        self.navigation_started = False
        self.navigation_finished = False
        self.navigation_status = None

        self.set_navigator_enabled(True)

        if not self.wait_for_navigation_start():

            self.set_navigator_enabled(False)

            self.get_logger().error(
                'Nav2 no inició navegación'
            )

            self.write_result(
                position_id,
                True,
                'NOT_STARTED',
                0.0,
                goal
            )

            return

        # Una vez aceptada la meta bloqueamos
        # nuevos envíos semánticos.
        self.set_navigator_enabled(False)

        # Comienza aquí la medición geométrica.
        # El teleport inicial NO entra en distance_traveled_m.
        self.reset_navigation_metrics()

        # Inicio de navegación según /clock.
        navigation_start_sim_ns = (
            self.get_clock().now().nanoseconds
        )

        # También medimos tiempo real para diagnóstico del RTF.
        navigation_start_wall = (
            time.monotonic()
        )

        status, timeout_reason = (
            self.wait_for_navigation_result()
        )

        navigation_end_sim_ns = (
            self.get_clock().now().nanoseconds
        )

        navigation_time = max(
            0.0,
            (
                navigation_end_sim_ns
                - navigation_start_sim_ns
            )
            / 1_000_000_000.0
        )

        navigation_wall_time = (
            time.monotonic()
            - navigation_start_wall
        )

        if status is None:

            status_name = (
                timeout_reason
                or 'TIMEOUT'
            )

            self.get_logger().warning(
                'Navegación terminada por '
                f'{status_name}'
            )

            self.cancel_active_navigation()

        else:

            status_names = {
                4: 'SUCCEEDED',
                5: 'CANCELED',
                6: 'ABORTED',
            }

            status_name = status_names.get(
                status,
                f'UNKNOWN_{status}'
            )

        self.get_logger().info(
            'Tiempo navegación | '
            f'sim={navigation_time:.3f} s | '
            f'real={navigation_wall_time:.3f} s'
        )

        self.get_logger().info(
            'Resultado navegación: '
            f'{status_name}'
        )

        self.write_result(
            position_id,
            True,
            status_name,
            navigation_time,
            goal,
            navigation_time_real=(
                navigation_wall_time
            ),
            distance_traveled_m=(
                self.navigation_distance_m
            )
        )


    # =============================================================
    # CSV
    # =============================================================

    def write_result(
        self,
        position_id,
        detection_success,
        navigation_status,
        navigation_time,
        goal,
        navigation_time_real=0.0,
        distance_traveled_m=0.0
    ):

        goal_x = ''
        goal_y = ''

        target_x = ''
        target_y = ''
        target_z = ''
        target_error_m = ''
        target_correct = ''

        target_world_x = ''
        target_world_y = ''
        target_world_z = ''

        localization_error_3d_center_m = ''
        localization_error_3d_bbox_m = ''

        mask_iou = ''
        mask_sync_dt_s = ''
        mask_pred_area_px = ''
        mask_gt_area_px = ''
        mask_intersection_px = ''
        mask_union_px = ''
        mask_gt_instance = ''

        if (
            detection_success
            and self.episode_mask_metrics is not None
        ):

            metric = self.episode_mask_metrics

            mask_iou = round(
                float(
                    metric['mask_iou']
                ),
                4
            )

            mask_sync_dt_s = round(
                float(
                    metric['mask_sync_dt_s']
                ),
                4
            )

            mask_pred_area_px = int(
                metric['mask_pred_area_px']
            )

            mask_gt_area_px = int(
                metric['mask_gt_area_px']
            )

            mask_intersection_px = int(
                metric[
                    'mask_intersection_px'
                ]
            )

            mask_union_px = int(
                metric['mask_union_px']
            )

            mask_gt_instance = str(
                metric['mask_gt_instance']
            )

        if (
            detection_success
            and self.latest_target_point is not None
        ):

            target_x = float(
                self.latest_target_point.point.x
            )

            target_y = float(
                self.latest_target_point.point.y
            )

            target_z = float(
                self.latest_target_point.point.z
            )

            if self.ground_truth is not None:

                (
                    target_correct,
                    target_error_value,
                    target_bbox_error_value,
                    target_world_x,
                    target_world_y,
                    target_world_z,
                    expected_instance,
                    xmin,
                    xmax,
                    ymin,
                    ymax,
                ) = self.evaluate_multi_instance_target(
                    position_id,
                    target_x,
                    target_y,
                    target_z
                )

                localization_error_3d_center_m = round(
                    target_error_value,
                    3
                )

                localization_error_3d_bbox_m = round(
                    target_bbox_error_value,
                    3
                )

                target_error_m = round(
                    target_error_value,
                    3
                )

                self.get_logger().info(
                    'Evaluación multi-instancia | '
                    f'posición={position_id} | '
                    f'esperado={expected_instance} | '
                    f'target_odom=('
                    f'{target_x:.3f}, '
                    f'{target_y:.3f}) | '
                    f'target_world=('
                    f'{target_world_x:.3f}, '
                    f'{target_world_y:.3f}, '
                    f'{target_world_z:.3f}) | '
                    f'GT X=['
                    f'{xmin:.3f}, '
                    f'{xmax:.3f}] | '
                    f'Y=['
                    f'{ymin:.3f}, '
                    f'{ymax:.3f}] | '
                    f'correcto={target_correct} | '
                    f'error_centro_3d='
                    f'{target_error_value:.3f} m | '
                    f'error_bbox_3d='
                    f'{target_bbox_error_value:.3f} m'
                )

            else:

                dx = (
                    target_x
                    - self.target_reference['x']
                )

                dy = (
                    target_y
                    - self.target_reference['y']
                )

                dz = (
                    target_z
                    - self.target_reference['z']
                )

                target_error_value = math.sqrt(
                    dx * dx
                    + dy * dy
                    + dz * dz
                )

                target_error_m = round(
                    target_error_value,
                    3
                )

                target_correct = int(
                    self.target_region['xmin']
                    <= target_x
                    <= self.target_region['xmax']
                    and
                    self.target_region['ymin']
                    <= target_y
                    <= self.target_region['ymax']
                )

                self.get_logger().info(
                    'Evaluación de target | '
                    f'X={target_x:.3f}, '
                    f'Y={target_y:.3f} | '
                    f'región X=['
                    f'{self.target_region["xmin"]:.3f}, '
                    f'{self.target_region["xmax"]:.3f}] | '
                    f'Y=['
                    f'{self.target_region["ymin"]:.3f}, '
                    f'{self.target_region["ymax"]:.3f}] | '
                    f'correcto={target_correct} | '
                    f'error_centro_3d='
                    f'{target_error_value:.3f} m'
                )

        if goal is not None:

            goal_x = (
                goal.pose.position.x
            )

            goal_y = (
                goal.pose.position.y
            )

        # -------------------------------------------------
        # Métricas finales de navegación
        # -------------------------------------------------

        final_robot_x = ''
        final_robot_y = ''
        final_goal_error_m = ''
        goal_region_reached = ''

        final_xy = self.navigation_final_xy

        # Si no hubo navegación, intentamos al menos registrar
        # la posición actual para diagnóstico.
        if final_xy is None:
            final_xy = self.get_robot_xy_odom()

        if final_xy is not None:

            final_robot_x = round(
                float(final_xy[0]),
                3
            )

            final_robot_y = round(
                float(final_xy[1]),
                3
            )

            if goal is not None:

                goal_error = math.hypot(
                    float(goal.pose.position.x)
                    - float(final_xy[0]),
                    float(goal.pose.position.y)
                    - float(final_xy[1])
                )

                final_goal_error_m = round(
                    goal_error,
                    3
                )

                goal_region_reached = int(
                    goal_error
                    <= self.goal_region_tolerance_m
                )

        navigation_timeout = int(
            navigation_status in (
                'TIMEOUT',
                'WALL_WATCHDOG',
            )
        )

        # -------------------------------------------------
        # Clasificación del fallo
        # -------------------------------------------------

        failure_stage = ''
        failure_reason = ''

        if not detection_success:

            failure_stage = 'PERCEPTION'
            failure_reason = 'NO_DETECTION'

        elif target_correct == 0:

            failure_stage = 'PERCEPTION'
            failure_reason = 'WRONG_TARGET_INSTANCE'

        elif target_correct == '':

            failure_stage = 'PERCEPTION'
            failure_reason = 'TARGET_POINT_MISSING'

        elif navigation_status == 'NOT_STARTED':

            failure_stage = 'NAVIGATION'
            failure_reason = 'NOT_STARTED'

        elif navigation_status == 'TIMEOUT':

            failure_stage = 'NAVIGATION'
            failure_reason = 'TIMEOUT'

        elif navigation_status == 'WALL_WATCHDOG':

            failure_stage = 'NAVIGATION'
            failure_reason = 'WALL_WATCHDOG'

        elif navigation_status == 'ABORTED':

            failure_stage = 'NAVIGATION'
            failure_reason = 'ABORTED'

        elif navigation_status == 'CANCELED':

            failure_stage = 'NAVIGATION'
            failure_reason = 'CANCELED'

        elif (
            navigation_status == 'SUCCEEDED'
            and goal_region_reached == 0
        ):

            failure_stage = 'NAVIGATION'
            failure_reason = (
                'SUCCEEDED_OUTSIDE_GOAL_REGION'
            )

        pose_config = self.positions[
            position_id
        ]

        episode_id = (
            f'{self.object_name}_'
            f'{self.condition}_'
            f'{position_id}'
        )

        with open(
            self.csv_path,
            'a',
            newline='',
            encoding='utf-8'
        ) as csv_file:

            writer = csv.writer(
                csv_file
            )

            writer.writerow([
                episode_id,
                position_id,
                self.scenario,
                self.object_name,
                self.condition,
                pose_config['x'],
                pose_config['y'],
                int(detection_success),
                target_x,
                target_y,
                target_z,
                target_error_m,
                target_correct,
                (
                    round(target_world_x, 3)
                    if target_world_x != ''
                    else ''
                ),
                (
                    round(target_world_y, 3)
                    if target_world_y != ''
                    else ''
                ),
                (
                    round(target_world_z, 3)
                    if target_world_z != ''
                    else ''
                ),
                localization_error_3d_center_m,
                localization_error_3d_bbox_m,
                mask_iou,
                mask_sync_dt_s,
                mask_pred_area_px,
                mask_gt_area_px,
                mask_intersection_px,
                mask_union_px,
                mask_gt_instance,
                navigation_status,
                round(
                    navigation_time,
                    3
                ),
                goal_x,
                goal_y,
                round(
                    navigation_time_real,
                    3
                ),
                round(
                    distance_traveled_m,
                    3
                ),
                final_robot_x,
                final_robot_y,
                final_goal_error_m,
                goal_region_reached,
                self.goal_region_tolerance_m,
                navigation_timeout,
                failure_stage,
                failure_reason,
            ])


    # =============================================================
    # RUN
    # =============================================================

    def run(self):

        clients = [
            (
                self.simulation_state_client,
                '/set_simulation_state'
            ),
            (
                self.get_entity_state_client,
                '/get_entity_state'
            ),
            (
                self.set_entity_state_client,
                '/set_entity_state'
            ),
            (
                self.local_costmap_client,
                'local costmap'
            ),
            (
                self.global_costmap_client,
                'global costmap'
            ),
            (
                self.reset_target_client,
                '/semantic_object/reset_target'
            ),
            (
                self.navigator_parameter_client,
                '/semantic_navigator/set_parameters'
            ),
            (
                self.cancel_navigation_client,
                '/navigate_to_pose/_action/cancel_goal'
            ),
        ]

        for client, name in clients:

            self.wait_for_service(
                client,
                name
            )

        self.set_navigator_enabled(
            False
        )

        # Comenzamos siempre desde PAUSE.
        self.set_simulation_state(
            2
        )

        self.get_logger().info(
            'Secuencia experimental: '
            + ' -> '.join(
                self.position_ids
            )
        )

        for position_id in self.position_ids:

            self.run_episode(
                position_id
            )

        self.set_navigator_enabled(
            False
        )

        # Dejamos el escenario listo
        # nuevamente en P1.
        self.set_simulation_state(
            2
        )

        self.restore_robot(
            'P1'
        )

        self.clear_costmaps()

        self.reset_semantic_target()

        self.get_logger().info(
            'Robot restaurado a P1.'
        )

        self.get_logger().info(
            '================================'
        )

        self.get_logger().info(
            'EVALUACIÓN TERMINADA'
        )

        self.get_logger().info(
            f'CSV: {self.csv_path}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = EvaluationRunner()

    try:
        node.run()

    except KeyboardInterrupt:
        node.get_logger().warning(
            'Evaluación interrumpida'
        )

    except Exception as error:
        node.get_logger().error(
            f'Error en evaluation_runner: {error}'
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()