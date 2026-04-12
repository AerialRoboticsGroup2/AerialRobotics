import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class HSVTunerTello(Node):
    def __init__(self):
        super().__init__('hsv_tuner_tello')
        
        # Define QoS profile compatible with the real drone (High frequency, Best Effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Using generic topic, overridable by namespace
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.camera_callback, qos_profile)
        self.bridge = CvBridge()
        
        # Create a window for trackbars
        cv2.namedWindow('Tuning')
        cv2.createTrackbar('H Min', 'Tuning', 40, 179, self.nothing)
        cv2.createTrackbar('S Min', 'Tuning', 40, 255, self.nothing)
        cv2.createTrackbar('V Min', 'Tuning', 40, 255, self.nothing)
        cv2.createTrackbar('H Max', 'Tuning', 80, 179, self.nothing)
        cv2.createTrackbar('S Max', 'Tuning', 255, 255, self.nothing)
        cv2.createTrackbar('V Max', 'Tuning', 255, 255, self.nothing)

        self.get_logger().info("HSV Tuner gestart! Gebruik de schuifregelaars op de 'Tuning' venster.")

    def nothing(self, x):
        pass

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Get trackbar positions
            h_min = cv2.getTrackbarPos('H Min', 'Tuning')
            s_min = cv2.getTrackbarPos('S Min', 'Tuning')
            v_min = cv2.getTrackbarPos('V Min', 'Tuning')
            h_max = cv2.getTrackbarPos('H Max', 'Tuning')
            s_max = cv2.getTrackbarPos('S Max', 'Tuning')
            v_max = cv2.getTrackbarPos('V Max', 'Tuning')

            lower_bound = np.array([h_min, s_min, v_min])
            upper_bound = np.array([h_max, s_max, v_max])

            mask = cv2.inRange(hsv, lower_bound, upper_bound)
            result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            # Show results
            cv2.imshow("Original", cv_image)
            cv2.imshow("Mask", mask)
            cv2.imshow("Result", result)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Fout: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HSVTunerTello()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
