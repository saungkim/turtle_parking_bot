import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import time

# === 사용자 정의 ===
x1=369 
y1=1 
x2=480 
y2=72
# w = x2-x1
# h = y2-y1
ENTRANCE_ROI = (x1, y1, x2, y2)  # 입구 영역 (x1, y1, x2, y2)

p1=(296,215)
PARKING_SPOTS = {
    'A1': p1,
    # 'A2': (300, 100),
    # 'B1': (100, 300),
    # 'B2': (300, 300),
}  # 맵 상의 고정된 좌표 (이미지 기준)
# YOLO_MODEL_PATH = '/home/shin/bacass.pt'
YOLO_MODEL_PATH = '/home/rokey/yolov11/best.pt'

class ParkingMonitor(Node):
    def __init__(self):
        super().__init__('parking_monitor')
        self.bridge = CvBridge()
        self.model = YOLO(YOLO_MODEL_PATH)
        self.image_sub = self.create_subscription(Image, '/usb_camera/image_raw', self.image_callback, 10)
        self.entry_pub = self.create_publisher(Bool, '/parking/entry', 10)
        # self.empty_pub = self.create_publisher(PoseStamped, '/parking/empty_spot', 10)
        self.last_entry_time = 0
        self.entry_cooldown = 3.0  # seconds

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)[0]
        car_detected = False

        for box in results.boxes:
            cls = int(box.cls[0])
            if self.model.names[cls] != 'car':
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = int((x1 + x2)/2), int((y1 + y2)/2)

            # 입구 영역 감지
            if ENTRANCE_ROI[0] <= cx <= ENTRANCE_ROI[2] and ENTRANCE_ROI[1] <= cy <= ENTRANCE_ROI[3]:
                if time.time() - self.last_entry_time > self.entry_cooldown:
                    self.get_logger().info("차량 입차 감지")
                    car_detected = True
                    self.entry_pub.publish(Bool(data=car_detected))
                    self.last_entry_time = time.time()

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