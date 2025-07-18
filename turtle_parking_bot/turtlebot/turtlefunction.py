import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class TurtleFunction(Node):
    def __init__(self):
        super().__init__('talker')
        self.run_rosbridge()
        self.publisher_ = self.create_publisher(String, '/chatter', 10)
        timer_period = 1.0  # 1초마다 메시지 전송
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from ROS2 Python node (VS Code)'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

    def run_rosbridge(self):
        subprocess.run([
            "ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"
        ])

def main(args=None):
    rclpy.init(args=args)
    node = TurtleFunction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
