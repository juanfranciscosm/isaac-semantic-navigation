#!/usr/bin/env python3

import time
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class NavigationVelocityMonitor(Node):

    def __init__(self) -> None:
        super().__init__('navigation_velocity_monitor')

        self.start_time = time.monotonic()

        self.values: Dict[str, Dict[str, float | None]] = {
            'nav': self.empty_record(),
            'smooth': self.empty_record(),
            'final': self.empty_record(),
            'odom': self.empty_record(),
        }

        self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            lambda msg: self.twist_callback('nav', msg),
            10,
        )

        self.create_subscription(
            Twist,
            '/cmd_vel_smoothed',
            lambda msg: self.twist_callback('smooth', msg),
            10,
        )

        self.create_subscription(
            Twist,
            '/cmd_vel',
            lambda msg: self.twist_callback('final', msg),
            10,
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        self.create_timer(0.25, self.print_status)

    @staticmethod
    def empty_record() -> Dict[str, float | None]:
        return {
            'vx': 0.0,
            'wz': 0.0,
            'max_abs_vx': 0.0,
            'max_abs_wz': 0.0,
            'timestamp': None,
        }

    def update_record(
        self,
        source: str,
        linear_x: float,
        angular_z: float,
    ) -> None:
        record = self.values[source]

        record['vx'] = float(linear_x)
        record['wz'] = float(angular_z)

        record['max_abs_vx'] = max(
            float(record['max_abs_vx']),
            abs(float(linear_x)),
        )

        record['max_abs_wz'] = max(
            float(record['max_abs_wz']),
            abs(float(angular_z)),
        )

        record['timestamp'] = time.monotonic()

    def twist_callback(
        self,
        source: str,
        message: Twist,
    ) -> None:
        self.update_record(
            source,
            message.linear.x,
            message.angular.z,
        )

    def odom_callback(self, message: Odometry) -> None:
        self.update_record(
            'odom',
            message.twist.twist.linear.x,
            message.twist.twist.angular.z,
        )

    def format_source(self, source: str) -> str:
        record = self.values[source]
        timestamp = record['timestamp']

        if timestamp is None:
            age = '--'
        else:
            age = f'{time.monotonic() - float(timestamp):.2f}s'

        return (
            f'{source:<6} '
            f'vx={float(record["vx"]):+6.3f} '
            f'wz={float(record["wz"]):+6.3f} '
            f'age={age}'
        )

    def print_status(self) -> None:
        elapsed = time.monotonic() - self.start_time

        print(
            f't={elapsed:7.2f} | '
            f'{self.format_source("nav")} | '
            f'{self.format_source("smooth")} | '
            f'{self.format_source("final")} | '
            f'{self.format_source("odom")}',
            flush=True,
        )

    def print_summary(self) -> None:
        print()
        print('========== RESUMEN DE VELOCIDADES ==========')

        for source in ('nav', 'smooth', 'final', 'odom'):
            record = self.values[source]

            print(
                f'{source:<6} | '
                f'max |vx|={float(record["max_abs_vx"]):.3f} m/s | '
                f'max |wz|={float(record["max_abs_wz"]):.3f} rad/s'
            )


def main() -> None:
    rclpy.init()

    node = NavigationVelocityMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.print_summary()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
