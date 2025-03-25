import rclpy
import os
from pygame import mixer, error
from rclpy.node import Node
from wall_e_msg_srv.srv import PlaySound
from wall_e_msg_srv.srv import SetVolume

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_DIR = os.path.abspath(os.path.join(PACKAGE_DIR, "../../../../../.."))
WEB_DIR = os.path.join(WORKSPACE_DIR, "web_server")
DEFAULT_SOUNDS_PATH = os.path.join(WEB_DIR, "sounds")

DEVICE_NAME = "bcm2835 Headphones, bcm2835 Headphones"

DEFAULT_VOLUME = 0.2


class SoundBoxNode(Node):
    def __init__(self):
        super().__init__("soundbox_node")

        self.declare_parameter("default_volume", DEFAULT_VOLUME)

        default_volume = self.get_parameter("default_volume").get_parameter_value().double_value

        self.path = DEFAULT_SOUNDS_PATH
        if not os.path.exists(self.path):
            raise FileNotFoundError(f"Sounds folder not found: {self.path}")

        try:
            mixer.init(buffer=2048, devicename=DEVICE_NAME)
        except error as e:
            self.get_logger().error(f"Failed to initialize pygame mixer: {e}")
            raise

        mixer.music.set_volume(default_volume)

        self.play_sound_srv = self.create_service(
            PlaySound, "play_sound", self.play_sound_callback
        )
        self.set_volume_srv = self.create_service(
            SetVolume, "set_volume", self.set_volume_callback
        )

    def play_sound(self, soundPath: str):
        mixer.music.load(soundPath)
        mixer.music.play()

    def set_volume(self, volume: int):
        mixer.music.set_volume(volume / 100)

    def play_sound_callback(self, request, response):
        sound_id = request.sound_id
        duration = request.duration

        self.get_logger().debug(
            f"play_sound_callback service called\nsound_id: {sound_id}, duration: {duration}"
        )

        sound_path = os.path.join(self.path, "sound" + str(sound_id) + ".mp3")
        self.play_sound(sound_path)
        response.success = True

        return response

    def set_volume_callback(self, request, response):
        volume = request.volume

        self.get_logger().debug(f"set_volume_callback service called\nvolume: {volume}")

        self.set_volume(volume)
        response.success = True

        return response

    def cleanup(self):
        super().destroy_node()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SoundBoxNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested, stopping node...")
    except:
        pass
    finally:
        node.cleanup()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
