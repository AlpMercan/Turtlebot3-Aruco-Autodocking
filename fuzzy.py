#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import time
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class FuzzyController:
    def __init__(self):
        # Define the universe of possible input and output values
        error = ctrl.Antecedent(np.arange(-1000, 1000, 1), 'error')
        delta_error = ctrl.Antecedent(np.arange(-100, 100, 1), 'delta_error')
        output = ctrl.Consequent(np.arange(-1, 1, 0.01), 'output')

        # Membership functions
        error['negative'] = fuzz.trimf(error.universe, [-1000, -500, 0])
        error['zero'] = fuzz.trimf(error.universe, [-500, 0, 500])
        error['positive'] = fuzz.trimf(error.universe, [0, 500, 1000])
        delta_error['negative'] = fuzz.trimf(delta_error.universe, [-100, -50, 0])
        delta_error['zero'] = fuzz.trimf(delta_error.universe, [-50, 0, 50])
        delta_error['positive'] = fuzz.trimf(delta_error.universe, [0, 50, 100])
        output['negative'] = fuzz.trimf(output.universe, [-1, -0.5, 0])
        output['zero'] = fuzz.trimf(output.universe, [-0.5, 0, 0.5])
        output['positive'] = fuzz.trimf(output.universe, [0, 0.5, 1])

        # Rules
        rule1 = ctrl.Rule(error['negative'] | delta_error['negative'], output['positive'])
        rule2 = ctrl.Rule(error['zero'], output['zero'])
        rule3 = ctrl.Rule(error['positive'] | delta_error['positive'], output['negative'])

        # Control system
        self.control_system = ctrl.ControlSystem([rule1, rule2, rule3])
        self.control_simulation = ctrl.ControlSystemSimulation(self.control_system)

    def update(self, current_error, current_delta_error):
        print(f"Current Error: {current_error}, Delta Error: {current_delta_error}")  # Debugging line
        self.control_simulation.input['error'] = current_error
        self.control_simulation.input['delta_error'] = current_delta_error
        try:
            self.control_simulation.compute()
            return self.control_simulation.output['output']
        except Exception as e:
            print(f"Error in Fuzzy Controller: {e}")
        # Implement fallback strategy here
            return 0  # Example fallback: return zero output


class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()
        self.angular_fuzzy = FuzzyController()
        self.linear_fuzzy = FuzzyController()
        self.last_error_angular = 0
        self.last_error_linear = 0
        self.last_time = time.time()

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

                # Angular Control
                angular_error = (cv_image.shape[1] / 2) - center_x
                delta_angular_error = angular_error - self.last_error_angular
                self.last_error_angular = angular_error
                angular_output = self.angular_fuzzy.update(angular_error, delta_angular_error)

                # Linear Control
                linear_error = marker_size - 5000  # Define desired_marker_size based on your setup
                delta_linear_error = linear_error - self.last_error_linear
                self.last_error_linear = linear_error
                linear_output = self.linear_fuzzy.update(linear_error, delta_linear_error)

                twist = Twist()
                twist.angular.z = angular_output
                twist.linear.x = -linear_output
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()

        except Exception as e:
            rospy.logerr("Error in camera_callback: %s", e)
            self.stop_robot()

    def calculate_marker_size(self, corner):
        width = corner[1][0] - corner[0][0]
        height = corner[2][1] - corner[0][1]
        return width * height

    def calculate_marker_center(self, corner):
        center_x = (corner[0][0] + corner[2][0]) / 2
        center_y = (corner[0][1] + corner[2][1]) / 2
        return center_x, center_y

    def stop_robot(self):
        twist = Twist()
        twist.angular.z = 0
        twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)

    def on_shutdown(self):
        rospy.loginfo("Shutting down. Stopping robot...")
        self.stop_robot()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = ArucoDetector()
    detector.run()
