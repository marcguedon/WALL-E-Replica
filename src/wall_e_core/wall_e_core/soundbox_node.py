import rclpy
from rclpy.node import Node
from pygame import mixer
from wall_e_msg_srv.srv import PlaySound


class SoundBoxNode(Node):
    def __init__(self):
        super().__init__("soundbox_node")

        mixer.init()
        mixer.music.set_volume(0.2)

        self.serv = self.create_service(
            PlaySound, "play_sound", self.play_sound_callback
        )

    def play_sound(self, soundPath: str):
        mixer.music.load(soundPath)
        mixer.music.play()

    def set_volume(self, volume: float):
        mixer.music.set_volume(volume)

    def play_sound_callback(self, request, response):
        sound_path = request.sound_file
        self.play_sound(sound_path)
        response.success = True

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SoundBoxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
