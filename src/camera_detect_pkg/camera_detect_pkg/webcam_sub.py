import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# 红色区间（HSV）
lower_red = np.array([0, 90, 128])
upper_red = np.array([180, 255, 255])

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('webcam_sub')
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) < 150:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(cv_image, (x+w//2, y+h//2), 5, (0, 255, 0), -1)

        cv2.imshow("detected", cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()