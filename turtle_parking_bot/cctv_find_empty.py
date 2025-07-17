import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

# === 사용자 정의 ROI ===
A1_ROI = (372, 171, 527, 257)
B3_ROI = (525, 141, 629, 204)

PARKING_SPOTS = {
    'A1': (296, 215),
    'B3': (381, 164),
}

YOLO_MODEL_PATH = '/home/rokey/yolov11/best.pt'

class ParkingMonitor(Node):
    def __init__(self):
        super().__init__('parking_monitor')
        self.bridge = CvBridge()
        self.model = YOLO(YOLO_MODEL_PATH)
        self.latest_frame = None

        # 이미지 구독
        self.image_sub = self.create_subscription(Image, '/usb_camera/image_raw', self.image_callback, 10)

        # 입차 감지 신호 구독
        self.entry_sub = self.create_subscription(Bool, '/parking/entry', self.entry_callback, 10)

        # 빈 자리 문자 퍼블리셔
        self.spot_pub = self.create_publisher(String, '/parking/empty_spot_id', 10)

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def entry_callback(self, msg):
        if not msg.data:
            return  # False일 경우 무시

        if self.latest_frame is None:
            self.get_logger().warn("이미지 수신 대기")
            return

        results = self.model(self.latest_frame)[0]
        occupied_spots = []

        for box in results.boxes:
            cls = int(box.cls[0])
            if self.model.names[cls] != 'car':
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

            if A1_ROI[0] <= cx <= A1_ROI[2] and A1_ROI[1] <= cy <= A1_ROI[3]:
                self.get_logger().info("차량이 A1 구역에 있습니다.")
                occupied_spots.append('A1')

            if B3_ROI[0] <= cx <= B3_ROI[2] and B3_ROI[1] <= cy <= B3_ROI[3]:
                self.get_logger().info("차량이 B3 구역에 있습니다.")
                occupied_spots.append('B3')
            

        if len(occupied_spots) == 2:
            self.get_logger().info("빈 공간 없음")
        else:
            for spot_id in PARKING_SPOTS.keys():
                if spot_id not in occupied_spots:
                    msg = String()
                    msg.data = spot_id
                    self.get_logger().info(f"빈 자리 발견: {spot_id} → 문자열 전송")
                    self.spot_pub.publish(msg)
                    break

def main(args=None):
    rclpy.init(args=args)
    node = ParkingMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
