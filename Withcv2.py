#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()
        self.pid_angular = PIDController(kp=0.000000000000000003, ki=0.05, kd=0.1)
        self.pid_linear = PIDController(kp=1.0, ki=0.1, kd=0.05)
        self.target_distance = 1000.0  # Desired distance from marker in pixels
        self.marker_physical_size = 0.2  # Physical size of the marker in meters (adjust as per your marker)
        self.focal_length = 800  # Example value, adjust based on your camera

        self.last_time = time.time()
        rospy.on_shutdown(self.on_shutdown)

    def camera_callback(self, data):
        try:
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            corners, ids, rejectedCandidates = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                target_corner = corners[0][0]
                center_x, center_y = self.calculate_marker_center(target_corner)
                marker_size = self.calculate_marker_size(target_corner)
                horizontal_error = (cv_image.shape[1] / 2) - center_x
                vertical_error = self.calculate_marker_distance(marker_size) - self.target_distance

                angular_z = self.pid_angular.update(horizontal_error, dt)
                linear_x = self.pid_linear.update(vertical_error, dt)

                twist = Twist()
                twist.angular.z = angular_z
                twist.linear.x = max(min(linear_x, 0.5), -0.5)  # Limiting linear speed
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()

        except Exception as e:
            print("Error in camera_callback: ", e)
            self.stop_robot()

    def calculate_marker_size(self, corner):
        width = corner[1][0] - corner[0][0]
        height = corner[2][1] - corner[0][1]
        return width * height

    def calculate_marker_center(self, corner):
        center_x = (corner[0][0] + corner[2][0]) / 2
        center_y = (corner[0][1] + corner[2][1]) / 2
        return center_x, center_y

    def calculate_marker_distance(self, marker_size):
        distance = (self.marker_physical_size * self.focal_length) / marker_size
        return distance

    def stop_robot(self):
        twist = Twist()
        twist.angular.z = 0
        twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)

    def on_shutdown(self):
        print("Shutting down. Stopping robot...")
        self.stop_robot()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = ArucoDetector()
    detector.run()
