import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class CentralController(Node):
    def __init__(self):
        super().__init__('central_controller')
        self.empty_spots = []  
        self.used_spots = set()
        self.spot_sub = self.create_subscription(PoseStamped, '/parking/empty_spot', self.spot_callback, 10)
        self.result_sub = self.create_subscription(String, '/robot/result', self.result_callback, 10)
        self.command_pub = self.create_publisher(PoseStamped, '/robot/target_spot', 10)
        self.waiting_for_result = False

    def spot_callback(self, msg):
        if self.waiting_for_result:
            return  
        key = (msg.pose.position.x, msg.pose.position.y)
        if key in self.used_spots:
            return
        self.get_logger().info(f"빈 좌표 수신: {key}")
        self.empty_spots.append(msg)
        self.try_send_next()

    def try_send_next(self):
        while self.empty_spots:
            next_spot = self.empty_spots.pop(0)  # 나중에 주차구역 순서 정해지면 따로 추가로 정리
            key = (next_spot.pose.position.x, next_spot.pose.position.y)
            if key not in self.used_spots:
                self.used_spots.add(key)
                self.command_pub.publish(next_spot)
                self.get_logger().info(f"로봇에게 좌표 전송: {key}")
                self.waiting_for_result = True
                break

    def result_callback(self, msg):
        result = msg.data
        self.waiting_for_result = False
        if result == 'success':
            self.get_logger().info("주차 성공")
            rclpy.shutdown()
        elif result == 'fail':
            self.get_logger().warn("주차 실패 → 좌표 재전송")
            self.try_send_next()
        else:
            self.get_logger().warn("불필요한 좌표 전송")

def main(args=None):
    rclpy.init(args=args)
    node = CentralController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("중앙 제어 노드 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
