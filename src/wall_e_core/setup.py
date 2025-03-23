from setuptools import find_packages, setup
from glob import glob

package_name = "wall_e_core"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="marc",
    maintainer_email="marcguedon@gmail.com",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "web_server_node = wall_e_core.web_server_node:main",
            "screen_node = wall_e_core.screen_node:main",
            "battery_node = wall_e_core.battery_node:main",
            "camera_node = wall_e_core.camera_node:main",
            "ultrasonic_sensors_node = wall_e_core.ultrasonic_sensors_node:main",
            "lights_node = wall_e_core.lights_node:main",
            "motors_node = wall_e_core.motors_node:main",
            "soundbox_node = wall_e_core.soundbox_node:main",
            "servomotors_node = wall_e_core.servomotors_node:main",
            "automatic_mode_node = wall_e_core.automatic_mode_node:main",
        ],
    },
)
