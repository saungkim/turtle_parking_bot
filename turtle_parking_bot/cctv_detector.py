import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import time

# === 사용자 정의 ===
x1=3 
y1=10 
x2=326 
y2=192
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
YOLO_MODEL_PATH = '/home/shin/yolov11/best.pt'

class ParkingMonitor(Node):
    def __init__(self):
        super().__init__('parking_monitor')
        self.bridge = CvBridge()
        self.model = YOLO(YOLO_MODEL_PATH)
        self.image_sub = self.create_subscription(Image, '/usb_camera/image_raw', self.image_callback, 10)
        self.entry_pub = self.create_publisher(String, '/parking/entry', 10)
        self.empty_pub = self.create_publisher(PoseStamped, '/parking/empty_spot', 10)
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
                    self.entry_pub.publish(String(data='car_entered'))
                    self.last_entry_time = time.time()

        #     # 주차 공간 점유 판단
        #     for spot_id, (sx, sy) in PARKING_SPOTS.items():
        #         if abs(cx - sx) < 30 and abs(cy - sy) < 30:
        #             self.get_logger().info(f"{spot_id} 차량 있음")
        #             PARKING_SPOTS[spot_id] = None  # 비워진 걸 None 처리함

        # # 빈 자리 탐색 및 publish
        # for spot_id, coords in PARKING_SPOTS.items():
        #     if coords is not None:
        #         x, y = coords
        #         pose = PoseStamped()
        #         pose.header.frame_id = 'map'
        #         pose.header.stamp = self.get_clock().now().to_msg()
        #         pose.pose.position.x = x / 100.0
        #         pose.pose.position.y = y / 100.0
        #         pose.pose.orientation.w = 1.0
        #         self.get_logger().info(f"{spot_id} 자리가 비어있음 → 내비게이션에 전달")
        #         self.empty_pub.publish(pose)
        #         break  # 첫 번째 빈 자리만 publish


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