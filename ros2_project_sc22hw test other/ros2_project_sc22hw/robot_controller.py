import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
import signal
from action_msgs.msg import GoalStatus
from rclpy.exceptions import ROSInterruptException

class NavigationColorTracker(Node):
    def __init__(self):
        super().__init__('navigation_color_tracker')
        
        # Initialize navigation client
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Image processing
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rotation_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.forward_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.rate = self.create_rate(10)
        
        # Use provided waypoints
        self.waypoints = [
            {'position': (5.0, 3.0, 0.0)},
            {'position': (5.5, -10.0, 0.0)},
            {'position': (-7.0, -11.0, 0.0)},
            {'position': (-8.5, 1.0, 0.0)}
        ]
        
        self.track_corner_1 = True
        self.track_corner_2 = False
        self.track_corner_3 = False
        self.track_corner_4 = False
        
        self.tracking_1 = False
        self.tracking_2 = False
        self.tracking_3 = False
        self.tracking_4 = False
        
        # Color detection flags
        self.red_found = False
        self.green_found = False
        self.blue_found = False
        
        # Navigation tracking
        self.tracking = False
        self.send_goal_future = None
        
        # Blue object tracking
        self.diff_blue_centers = 0
        self.blue_position_fixed = False
        self.blue_contour_area = 0
        self.goal_reached = False
        
        self.sensitivity = 10
        
        self.get_logger().info("Navigation and color tracking node initialized")

    def send_goal(self, x, y, yaw=0):
        """Send a navigation goal to the action server"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        self.tracking = True

        self.action_client.wait_for_server()
        # Send the goal
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle response to the navigation goal request"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Process navigation result"""
        self.tracking = False
        
        if self.tracking_1:
            self.track_corner_1 = False
            self.track_corner_2 = True
            self.tracking_1 = False
            
        elif self.tracking_2:
            self.track_corner_2 = False
            self.track_corner_3 = True
            self.tracking_2 = False
            
        elif self.tracking_3:
            self.track_corner_3 = False
            self.track_corner_4 = True
            self.tracking_3 = False
            
        elif self.tracking_4:
            self.track_corner_4 = False
            self.track_corner_1 = True
            self.tracking_4 = False

    def feedback_callback(self, feedback_msg):
        """Process navigation feedback"""
        feedback = feedback_msg.feedback

    def stop(self):
        """Stop robot movement"""
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0
        desired_velocity.angular.z = 0.0
        
        self.cmd_vel_pub.publish(desired_velocity)
        self.rotation_pub.publish(desired_velocity)
        self.forward_pub.publish(desired_velocity)
        
        for _ in range(3):
            self.cmd_vel_pub.publish(desired_velocity)
            time.sleep(0.1)
    
    def rotate(self, num):
        """Rotate robot to scan environment"""
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.1
        desired_velocity.angular.z = 1.0
        
        for _ in range(100):  # Rotate for a brief moment
            if self.blue_found:
                desired_velocity.angular.z = 0.0
                desired_velocity.linear.x = 0.0
                self.rotation_pub.publish(desired_velocity)
                self.rate.sleep()
                break
            self.rotation_pub.publish(desired_velocity)
            self.rate.sleep()
    
    def fix_blue_position(self):
        """Align robot with blue object"""
        desired_velocity = Twist()
        
        while True:  # Align until centered
            # If blue cube is not centered
            if abs(self.diff_blue_centers) > 4:
                if self.diff_blue_centers > 0:
                    desired_velocity.angular.z = 0.1  # Turn left
                else:
                    desired_velocity.angular.z = -0.1  # Turn right
            else:
                self.blue_position_fixed = True
                break
            
            self.rotation_pub.publish(desired_velocity)
            self.rate.sleep()
    
    def move_forward(self):
        """Move forward toward blue object"""
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.5
        
        while True:
            self.forward_pub.publish(desired_velocity)
            self.rate.sleep()
            
            # Stop when contour area reaches threshold
            if self.blue_contour_area > 30000:  
                desired_velocity.linear.x = 0.0
                self.forward_pub.publish(desired_velocity)
                self.goal_reached = True
                break
    
    def image_callback(self, data):
        """Process camera images and detect colors"""
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            
            hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
            hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
            
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
            
            hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
            hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
            
            # Convert to HSV
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Create color masks
            blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
            green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
            red_mask = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
            
            # Find contours for each color
            contours_blue, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_green, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_red, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Process green contours
            if len(contours_green) > 0:
                c = max(contours_green, key=cv2.contourArea)
                self.check_contours(c, image, "green", hsv_image)
                if not self.green_found:
                    self.set_var("green", c, image)
            
            # Process red contours
            if len(contours_red) > 0:
                c = max(contours_red, key=cv2.contourArea)
                self.check_contours(c, image, "red", hsv_image)
                if not self.red_found:
                    self.set_var("red", c, image)
            
            # Process blue contours only if red and green are found
            if len(contours_blue) > 0:
                c = max(contours_blue, key=cv2.contourArea)
                if self.red_found and self.green_found:
                    self.check_contours(c, image, "blue", hsv_image)
                    if not self.blue_found:
                        self.set_var("blue", c, image)
            
            cv2.namedWindow('camera_Feed', cv2.WINDOW_NORMAL)
            cv2.imshow('camera_Feed', image)
            cv2.resizeWindow('camera_Feed', 320, 240)
            cv2.waitKey(3)  
            
        except Exception as e:
            self.get_logger().error(f"Error in image processing: {e}")
    
    def check_contours(self, c, image, colour, hsv_image):
        """Check contours and process them based on size"""
        # Minimum area threshold
        x = 500
        
        # Check if contour is large enough
        if cv2.contourArea(c) > x:
            # Store blue contour area for tracking
            if colour == "blue":
                self.blue_contour_area = cv2.contourArea(c)
            
            (x, y), radius = cv2.minEnclosingCircle(c)
            center = (int(x), int(y))
            radius = int(radius)
            
            color_map = {
                "red": (0, 0, 255),
                "green": (0, 255, 0),
                "blue": (255, 0, 0)
            }
            display_color = color_map.get(colour, (255, 255, 0))
            
            cv2.circle(image, center, radius, display_color, 4)
            
            # Calculate blue object position relative to camera center
            if self.blue_found and colour == "blue":
                M = cv2.moments(c)
                if M['m00'] != 0:
                    cX = int(M["m10"] / M["m00"])
                    img_width = image.shape[1]
                    camera_center_x = img_width / 2
                    self.diff_blue_centers = camera_center_x - cX
                    
                    # Add text showing area
                    area_text = f"Area: {self.blue_contour_area:.0f}"
                    cv2.putText(image, area_text, (center[0], center[1] - 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    def set_var(self, colour, c, image):
        """Set color detection flags and log detection"""
        if colour == "red":
            self.red_found = True
            self.get_logger().info("Red object found")
        elif colour == "green":
            self.green_found = True
            self.get_logger().info("Green object found")
        elif colour == "blue":
            self.blue_found = True
            self.get_logger().info("Blue object found")
            
            # Cancel navigation if active
            if self.send_goal_future and self.send_goal_future.done():
                self.get_logger().info("Canceling navigation to track blue object")
                goal_handle = self.send_goal_future.result()
                if goal_handle:
                    goal_handle.cancel_goal_async()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationColorTracker()
    
    # Set up signal handler
    def signal_handler(sig, frame):
        node.stop()
        rclpy.shutdown()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    try:
        while rclpy.ok():
            # If blue is not found yet
            if not node.blue_found:
                if not node.tracking:
                    # Move to corner 1
                    if node.track_corner_1:
                        node.tracking_1 = True
                        node.send_goal(5.0, 3.0, 0)
                        node.rotate(1)
                    # Move to corner 2
                    elif node.track_corner_2:
                        node.tracking_2 = True
                        node.send_goal(5.5, -10.0, 0)
                        node.rotate(2)
                    # Move to corner 3
                    elif node.track_corner_3:
                        node.tracking_3 = True
                        node.send_goal(-7.0, -11.0, 0)
                        node.rotate(3)
                    # Move to corner 4
                    elif node.track_corner_4:
                        node.tracking_4 = True
                        node.send_goal(-8.5, 1.0, 0)
                        node.rotate(4)
            # Blue is found, track it
            else:
                # Cancel current navigation goal if any
                if node.send_goal_future and node.send_goal_future.done():
                    if node.send_goal_future.result():
                        node.send_goal_future.result().cancel_goal_async()
                
                # Align with blue object
                if not node.blue_position_fixed:
                    node.fix_blue_position()
                
                # Move toward blue object
                if node.blue_position_fixed and not node.goal_reached:
                    node.stop()
                    node.move_forward()
    
    except ROSInterruptException:
        pass
    finally:
        node.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
