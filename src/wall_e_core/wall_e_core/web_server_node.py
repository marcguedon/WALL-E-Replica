import http.server
import socketserver
import rclpy
import threading
import os
from rclpy.node import Node

PORT = 80
PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_DIR = os.path.abspath(os.path.join(PACKAGE_DIR, "../../../../../../.."))
WEB_DIR = os.path.join(WORKSPACE_DIR, "web_server")

if not os.path.exists(WEB_DIR):
    raise FileNotFoundError(f"web_interface folder not found: {WEB_DIR}")


class WebServerNode(Node):
    def __init__(self):
        super().__init__("web_server_node")
        self.get_logger().info("Web server is starting...")
        self.server_thread = threading.Thread(target=self.run, daemon=True)
        self.server_thread.start()

    def run(self):
        os.chdir(WEB_DIR)
        handler = http.server.SimpleHTTPRequestHandler
        with socketserver.TCPServer(("", PORT), handler) as httpd:
            self.get_logger().info(f"Online web server on http://localhost:{PORT}")
            try:
                httpd.serve_forever()
            except BrokenPipeError:
                self.get_logger().warning(
                    "Client closed the connection before the server could finish sending the response."
                )
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
