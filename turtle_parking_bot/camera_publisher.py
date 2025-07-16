import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

def find_available_cameras(max_index=5):
    available = []
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                window_name = f"Camera {i}"
                resized = cv2.resize(frame, (320, 240))
                cv2.imshow(window_name, resized)
                available.append(i)
            cap.release()
    cv2.waitKey(1)
    return available

class USBCameraPublisher(Node):
    def __init__(self, cam_index):
        super().__init__('usb_camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'usb_camera/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(cam_index)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open USB camera at index {cam_index}')
            return

        self.get_logger().info(f'Using camera at index {cam_index}')
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Published image frame')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    print("🔍 사용 가능한 USB 카메라를 탐색 중입니다...")
    available = find_available_cameras()

    if not available:
        print("❌ 사용 가능한 카메라가 없습니다.")
        return

    print("\n✅ 사용 가능한 카메라:")
    for idx in available:
        print(f"  [{idx}] /dev/video{idx}")

    print("\n카메라 창에서 번호를 확인한 후 닫고, 사용할 카메라 번호를 입력하세요.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    selected = None
    while selected not in available:
        try:
            selected = int(input("➡ 사용할 카메라 번호 입력: "))
        except ValueError:
            continue

    rclpy.init(args=args)
    node = USBCameraPublisher(cam_index=selected)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
