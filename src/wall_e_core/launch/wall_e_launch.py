import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = (
        FindPackageShare("wall_e_core").find("wall_e_core") + "/config/params.yaml"
    )

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge",
        output="screen",
        parameters=[{"port": 9090}],
        respawn=True,
    )

    web_server_node = Node(
        package="wall_e_core",
        executable="web_server_node",
        name="web_server_node",
        output="screen",
        parameters=[config_file],
        respawn=True,
    )
    screen_node = Node(
        package="wall_e_core",
        executable="screen_node",
        name="screen_node",
        parameters=[config_file],
        respawn=True,
    )
    battery_node = Node(
        package="wall_e_core",
        executable="battery_node",
        name="battery_node",
        parameters=[config_file],
        respawn=True,
    )
    soundbox_node = Node(
        package="wall_e_core",
        executable="soundbox_node",
        name="soundbox_node",
        parameters=[config_file],
        respawn=True,
    )
    ultrasonic_sensors_node = Node(
        package="wall_e_core",
        executable="ultrasonic_sensors_node",
        name="ultrasonic_sensors_node",
        parameters=[config_file],
        respawn=True,
    )
    camera_node = Node(
        package="wall_e_core",
        executable="camera_node",
        name="camera_node",
        parameters=[config_file],
        respawn=True,
    )
    lights_node = Node(
        package="wall_e_core",
        executable="lights_node",
        name="lights_node",
        parameters=[config_file],
        respawn=True,
    )
    motors_node = Node(
        package="wall_e_core",
        executable="motors_node",
        name="motors_node",
        parameters=[config_file],
        respawn=True,
    )
    servomotors_node = Node(
        package="wall_e_core",
        executable="servomotors_node",
        name="servomotors_node",
        parameters=[config_file],
        respawn=True,
    )
    automatic_mode_node = Node(
        package="wall_e_core",
        executable="automatic_mode_node",
        name="automatic_mode_node",
        parameters=[config_file],
        respawn=True,
    )

    return launch.LaunchDescription(
        [
            lights_node,
            battery_node,
            ultrasonic_sensors_node,
            soundbox_node,
            camera_node,
            rosbridge_node,
            web_server_node,
            screen_node,
            motors_node,
            servomotors_node,
            automatic_mode_node,
        ]
    )
