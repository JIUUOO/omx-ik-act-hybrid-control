import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from rclpy.qos import qos_profile_sensor_data

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')

        self.declare_parameter('image_topic', '/webcam/image_raw')
        self.declare_parameter('world_points', [
            0.3, 0.2, 0.3, -0.2, 0.5, 0.2, 0.5, -0.2, 0.4, 0.0, 0.4, 0.1
        ])

        self.image_topic = self.get_parameter('image_topic').value
        world_points_flat = self.get_parameter('world_points').value
        
        # Convert flat list [x1,y1,x2,y2...] into list of [x,y] pairs
        self.world_pts = []
        for i in range(0, len(world_points_flat), 2):
            self.world_pts.append([world_points_flat[i], world_points_flat[i+1]])

        if len(self.world_pts) < 4:
            self.get_logger().error("Minimum 4 world points required for Homography!")
            return

        self.get_logger().info(f"Loaded {len(self.world_pts)} world points for calibration.")
        self.get_logger().info(f"Using NumPy {np.__version__} from {np.__file__}")
        self.get_logger().info(f"Using OpenCV {cv2.__version__} from {cv2.__file__}")

        self.bridge = CvBridge()
        # Use qos_profile_sensor_data (BEST_EFFORT) to match usb_cam
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.clicked_pts = []
        self.current_frame = None
        self.calibration_done = False
        
        cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)
        cv2.startWindowThread()
        cv2.setMouseCallback("Calibration", self.mouse_callback)
        self.show_waiting_frame()
        
        # GUI 업데이트를 위한 20Hz 타이머
        self.display_timer = self.create_timer(0.05, self.display_callback)
        
        self.get_logger().info("Please click the corresponding points on the OpenCV window.")

    def show_waiting_frame(self):
        wait_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(
            wait_image,
            f"Waiting for {self.image_topic}",
            (30, 240),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )
        cv2.imshow("Calibration", wait_image)
        cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and not self.calibration_done:
            if len(self.clicked_pts) < len(self.world_pts):
                self.clicked_pts.append([x, y])
                self.get_logger().info(f"Recorded point {len(self.clicked_pts)}: ({x}, {y})")
                
                if len(self.clicked_pts) == len(self.world_pts):
                    self.compute_homography()

    def compute_homography(self):
        self.calibration_done = True
        src_pts = np.array(self.clicked_pts, dtype=np.float32)
        dst_pts = np.array(self.world_pts, dtype=np.float32)

        # Use RANSAC for 6+ points to filter outliers and find best fit
        H, status = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC)
        
        if H is not None:
            # Flatten the matrix for easy copy-pasting into the YAML
            h_flat = H.flatten().tolist()
            h_formatted = ",\n      ".join([
                f"{h_flat[0]:.6f}, {h_flat[1]:.6f}, {h_flat[2]:.6f}",
                f"{h_flat[3]:.6f}, {h_flat[4]:.6f}, {h_flat[5]:.6f}",
                f"{h_flat[6]:.6f}, {h_flat[7]:.6f}, {h_flat[8]:.6f}"
            ])
            
            self.get_logger().info("\n=======================================================")
            self.get_logger().info("🎉 CALIBRATION COMPLETE 🎉")
            self.get_logger().info("=======================================================")
            self.get_logger().info("Please copy the following array into 'transform_params.yaml':\n")
            self.get_logger().info(f"    transform_matrix: [\n      {h_formatted}\n    ]")
            self.get_logger().info("=======================================================\n")
            self.get_logger().info("You can now press 'q' in the image window to close.")
        else:
            self.get_logger().error("Failed to compute Homography matrix. Points might be collinear.")

    def image_callback(self, msg: Image):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

    def display_callback(self):
        if self.current_frame is None:
            self.show_waiting_frame()
            return
            
        cv_image = self.current_frame.copy()
        
        # Draw target text
        if not self.calibration_done:
            pt_idx = len(self.clicked_pts)
            if pt_idx < len(self.world_pts):
                target = self.world_pts[pt_idx]
                text = f"Click point {pt_idx + 1}/{len(self.world_pts)}: X={target[0]}, Y={target[1]}"
                cv2.putText(cv_image, text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            cv2.putText(cv_image, "Calibration Done! Check Terminal.", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
        # Draw clicked points
        for i, pt in enumerate(self.clicked_pts):
            cv2.circle(cv_image, (int(pt[0]), int(pt[1])), 5, (0, 255, 0), -1)
            cv2.putText(cv_image, str(i+1), (int(pt[0])+10, int(pt[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Calibration", cv_image)
        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:  # 'q' or 'ESC'
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
