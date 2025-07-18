from parking_bot_interfaces.msg import EmptySpots  # ✅ 메시지 타입
import rclpy
from rclpy.node import Node
from turtle_parking_bot.emqx.emqx_pub import connect_mqtt, publish


class CenterController(Node):
    def __init__(self):
        super().__init__('center_controller')

        # topic Sub
        self.subscription = self.create_subscription(
            EmptySpots,
            '/parking/empty_spots_msg',  
            self.spot_callback,
            10
        )

        # MQTT 연결 초기화
        self.mqtt_client = connect_mqtt()
        self.mqtt_client.loop_start()

        self.get_logger().info("CenterController Node started. Listening to /parking/empty_spots_msg")

    def spot_callback(self, msg):
        zone_list = msg.spot_ids  # 이미 Python 리스트 형태임

        if not zone_list:
            self.get_logger().warn("수신된 spot_ids 리스트가 비어 있음")
            return

        zone_id = zone_list[0]  # 가장 앞에 있는 zone만 사용

        for robot_id in ["0", "2"]:
            payload = {
                "id": robot_id,
                "zone_id": zone_id,
                "type": "start"
            }
            publish(self.mqtt_client, payload)
            self.get_logger().info(f"MQTT 발송 → robot{robot_id}: {payload}")


def main(args=None):
    rclpy.init(args=args)
    node = CenterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료 요청 수신")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
