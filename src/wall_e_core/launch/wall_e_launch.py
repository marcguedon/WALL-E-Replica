import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    web_server_node = launch_ros.actions.Node(
        package="wall_e_core",
        executable="web_server_node",
        name="web_server_node",
        output="screen",
    )
    rosbridge_node = launch_ros.actions.Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge",
        output="screen",
        parameters=[{"port": 9090}],
    )
    screen_node = launch_ros.actions.Node(
        package="wall_e_core", executable="screen_node", name="screen_node"
    )
    battery_node = launch_ros.actions.Node(
        package="wall_e_core", executable="battery_node", name="battery_node"
    )
    soundbox_node = launch_ros.actions.Node(
        package="wall_e_core", executable="soundbox_node", name="soundbox_node"
    )
    ultrasonic_sensors_node = launch_ros.actions.Node(
        package="wall_e_core",
        executable="ultrasonic_sensors_node",
        name="ultrasonic_sensors_node",
    )
    camera_node = launch_ros.actions.Node(
        package="wall_e_core", executable="camera_node", name="camera_node"
    )
    lights_node = launch_ros.actions.Node(
        package="wall_e_core", executable="lights_node", name="lights_node"
    )
    motors_node = launch_ros.actions.Node(
        package="wall_e_core", executable="motors_node", name="motors_node"
    )
    servomotors_node = launch_ros.actions.Node(
        package="wall_e_core", executable="servomotors_node", name="servomotors_node"
    )

    return launch.LaunchDescription(
        [
            web_server_node,
            rosbridge_node,
            screen_node,
            battery_node,
            soundbox_node,
            ultrasonic_sensors_node,
            camera_node,
            lights_node,
            motors_node,
            servomotors_node,
        ]
    )
