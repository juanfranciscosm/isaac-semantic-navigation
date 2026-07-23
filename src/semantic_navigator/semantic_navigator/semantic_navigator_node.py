#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class SemanticNavigatorNode(Node):
    """
    Recibe una PoseStamped calculada por semantic_localizer
    y la envía al servidor NavigateToPose de Nav2.
    """

    def __init__(self):
        super().__init__('semantic_navigator')

        self.declare_parameter(
            'goal_topic',
            '/semantic_object/goal_pose'
        )

        self.declare_parameter(
            'action_name',
            '/navigate_to_pose'
        )

        # Comienza desactivado para hacer una prueba segura.
        self.declare_parameter(
            'enabled',
            False
        )

        # Evita volver a enviar metas prácticamente iguales.
        self.declare_parameter(
            'minimum_goal_change_m',
            0.40
        )

        # Mientras Nav2 navega, ignora las nuevas detecciones.
        self.declare_parameter(
            'ignore_new_goals_while_active',
            True
        )

        self.declare_parameter(
            'expected_frame',
            'odom'
        )

        self.goal_topic = str(
            self.get_parameter('goal_topic').value
        )

        self.action_name = str(
            self.get_parameter('action_name').value
        )

        self.minimum_goal_change_m = float(
            self.get_parameter(
                'minimum_goal_change_m'
            ).value
        )

        self.ignore_new_goals_while_active = bool(
            self.get_parameter(
                'ignore_new_goals_while_active'
            ).value
        )

        self.expected_frame = str(
            self.get_parameter('expected_frame').value
        )

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            self.action_name
        )

        self.goal_subscription = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10
        )

        self.goal_active = False
        self.goal_handle = None

        self.last_sent_goal: Optional[PoseStamped] = None

        self.last_feedback_log_ns = 0
        self.last_disabled_log_ns = 0
        self.server_ready_logged = False

        self.server_timer = self.create_timer(
            2.0,
            self.check_action_server
        )

        self.get_logger().info(
            'semantic_navigator_node iniciado'
        )

        self.get_logger().info(
            f'Escuchando metas en: {self.goal_topic}'
        )

        self.get_logger().info(
            f'Acción Nav2: {self.action_name}'
        )

        self.get_logger().info(
            'El envío automático comienza desactivado.'
        )

    def check_action_server(self):
        if (
            self.action_client.server_is_ready()
            and not self.server_ready_logged
        ):
            self.server_ready_logged = True

            self.get_logger().info(
                f'Servidor {self.action_name} disponible'
            )

    @staticmethod
    def planar_distance(
        first_goal: PoseStamped,
        second_goal: PoseStamped
    ) -> float:
        delta_x = (
            first_goal.pose.position.x
            - second_goal.pose.position.x
        )

        delta_y = (
            first_goal.pose.position.y
            - second_goal.pose.position.y
        )

        return math.hypot(
            delta_x,
            delta_y
        )

    def goal_callback(self, goal_message: PoseStamped):
        enabled = bool(
            self.get_parameter('enabled').value
        )

        if not enabled:
            now_ns = self.get_clock().now().nanoseconds

            # Evita imprimir el mismo mensaje en cada frame.
            if (
                now_ns - self.last_disabled_log_ns
                >= 5_000_000_000
            ):
                self.last_disabled_log_ns = now_ns

                self.get_logger().info(
                    'Meta semántica recibida, pero el nodo '
                    'está desactivado.'
                )

            return

        if goal_message.header.frame_id != self.expected_frame:
            self.get_logger().warning(
                'Meta rechazada por frame incorrecto: '
                f'{goal_message.header.frame_id}. '
                f'Se esperaba {self.expected_frame}.'
            )
            return

        if (
            self.goal_active
            and self.ignore_new_goals_while_active
        ):
            return

        if self.last_sent_goal is not None:
            goal_change = self.planar_distance(
                goal_message,
                self.last_sent_goal
            )

            if goal_change < self.minimum_goal_change_m:
                return

        if not self.action_client.server_is_ready():
            self.get_logger().warning(
                f'El servidor {self.action_name} '
                'todavía no está disponible.'
            )
            return

        navigation_goal = NavigateToPose.Goal()
        navigation_goal.pose = goal_message

        self.get_logger().info(
            'Enviando meta semántica a Nav2 | '
            f'x={goal_message.pose.position.x:.2f}, '
            f'y={goal_message.pose.position.y:.2f}, '
            f'frame={goal_message.header.frame_id}'
        )

        # Se bloquean temporalmente las metas siguientes.
        self.goal_active = True
        self.last_sent_goal = goal_message

        send_future = self.action_client.send_goal_async(
            navigation_goal,
            feedback_callback=self.feedback_callback
        )

        send_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        try:
            self.goal_handle = future.result()

        except Exception as error:
            self.goal_active = False
            self.last_sent_goal = None

            self.get_logger().error(
                f'Error enviando la meta: {error}'
            )
            return

        if not self.goal_handle.accepted:
            self.goal_active = False
            self.last_sent_goal = None

            self.get_logger().warning(
                'Nav2 rechazó la meta semántica'
            )
            return

        self.get_logger().info(
            'Nav2 aceptó la meta semántica'
        )

        result_future = (
            self.goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.result_callback
        )

    def feedback_callback(self, feedback_message):
        now_ns = self.get_clock().now().nanoseconds

        # Imprime feedback como máximo cada dos segundos.
        if (
            now_ns - self.last_feedback_log_ns
            < 2_000_000_000
        ):
            return

        self.last_feedback_log_ns = now_ns

        feedback = feedback_message.feedback

        self.get_logger().info(
            'Navegando hacia el objeto | '
            f'distancia restante='
            f'{feedback.distance_remaining:.2f} m'
        )

    def result_callback(self, future):
        self.goal_active = False

        try:
            wrapped_result = future.result()

        except Exception as error:
            self.get_logger().error(
                f'No se pudo leer el resultado: {error}'
            )
            return

        status = wrapped_result.status

        status_names = {
            GoalStatus.STATUS_SUCCEEDED: 'completada',
            GoalStatus.STATUS_ABORTED: 'abortada',
            GoalStatus.STATUS_CANCELED: 'cancelada',
        }

        status_name = status_names.get(
            status,
            f'estado desconocido {status}'
        )

        self.get_logger().info(
            f'Navegación {status_name}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SemanticNavigatorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()