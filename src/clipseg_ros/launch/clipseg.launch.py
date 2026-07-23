import glob
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_pythonpath():
    current_pythonpath = os.environ.get("PYTHONPATH", "")
    virtual_env = os.environ.get("VIRTUAL_ENV")

    if not virtual_env:
        return current_pythonpath

    candidates = glob.glob(
        os.path.join(
            virtual_env,
            "lib",
            "python*",
            "site-packages",
        )
    )

    if not candidates:
        return current_pythonpath

    venv_site_packages = candidates[0]

    if current_pythonpath:
        return f"{venv_site_packages}:{current_pythonpath}"

    return venv_site_packages


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic",
                default_value="/rgb",
            ),
            DeclareLaunchArgument(
                "prompt",
                default_value="car",
            ),
            DeclareLaunchArgument(
                "threshold",
                default_value="0.50",
            ),
            DeclareLaunchArgument(
                "max_rate_hz",
                default_value="2.0",
            ),
            Node(
                package="clipseg_ros",
                executable="clipseg_node",
                name="clipseg_node",
                output="screen",
                additional_env={
                    "PYTHONPATH": get_pythonpath(),
                },
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "prompt": LaunchConfiguration("prompt"),
                        "threshold": LaunchConfiguration("threshold"),
                        "max_rate_hz": LaunchConfiguration("max_rate_hz"),
                    }
                ],
            ),
        ]
    )