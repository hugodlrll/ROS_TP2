from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def create_team_group(team_color: str):
    return GroupAction(
        actions=[
            PushRosNamespace(team_color),
            Node(
                package="handball",
                executable="coach",
                name=f"coach_{team_color}",
            ),
            Node(
                package="handball",
                executable="player_node",
                name="player_0",
                parameters=[{"team": team_color, "id": 0}],
            ),
            Node(
                package="handball",
                executable="player_node",
                name="player_1",
                parameters=[{"team": team_color, "id": 1}],
            ),
            Node(
                package="handball",
                executable="player_node",
                name="player_2",
                parameters=[{"team": team_color, "id": 2}],
            ),
        ]
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("handball")
    urdf_file = os.path.join(pkg_share, "urdf", "multi_robots.urdf")

    with open(urdf_file, "r", encoding="utf-8") as urdf_handle:
        robot_description = urdf_handle.read()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                additional_env={"LIBGL_ALWAYS_SOFTWARE": "1"},
            ),
            create_team_group("blue"),
            create_team_group("red"),
            Node(
                package="handball",
                executable="referee",
                name="referee",
                output="screen",
                parameters=[{"match_duration": 600}],
            ),
            Node(
                package="handball",
                executable="ball",
                name="ball",
                output="screen",
            ),
        ]
    )
