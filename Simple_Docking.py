#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np



class Odom:
    def __init__(self) :
        rospy.init_node("Aruco_Centering", anonymous=True)
        self.fiducial_sub = rospy.Subscriber("/fiducial_transforms" , FiducialTransformArray , self.fiducial_callback)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.FinalPush)       
        self.pose_sub = rospy.Subscriber("/odom" , Odometry, self.rotate_robot)
        self.vel_pub = rospy.Publisher("/cmd_vel" , Twist, queue_size=10)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters_create()
        self.values_imported=False
        self.rotation_completed=False
        self.first_movementf= False
        self.init_flag=True
        self.rotation90=False
        self.last_flag=True
        self.dataflag=False
        self.starting_point_left=True
        self.Final_point=0
        self.target_angle=0
        self.first_distance=0
        self.final_distance=0
        self.kp=0.05
        self.yaw=0
        self.roll=0
        self.pitch=0
        self.linear_position_x=0.0
        self.linear_position_y=0.0
        self.linear_position_z=0.0
    
    def rotate_robot (self,msg):
            #for linear travel
            self.linear_position_x= msg.pose.pose.position.x
            self.linear_position_z= msg.pose.pose.position.z
            self.linear_position_y= msg.pose.pose.position.y
            #for angular travel
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

    def fiducial_callback(self,data):
        try:
            first_fiducial=data.transforms[0]
            self.dataflag=True

            if not self.values_imported:
                self.values_imported = True
                y=first_fiducial.transform.rotation.z
                x=first_fiducial.transform.translation.x
                z=first_fiducial.transform.translation.z
                if self.linear_position_y>0:
                    self.starting_point_left=True
                else:
                    self.starting_point_left=False
                self.target_angle=math.pi/2 - abs(y)
                self.first_distance=(x)
                self.final_distance=abs(z)
                print(f"target_angle = {self.target_angle}, first_distance = {self.first_distance}, final_distance = {self.final_distance}")
        except:
            self.dataflag=False
            rospy.logwarn_once("No fiducial array detected")
   
            
    def rotate_robot_paralel(self):
        if not self.rotation_completed and self.values_imported:
            rospy.loginfo("First Turn")

            command=Twist()
            if self.starting_point_left:
                command.angular.z=(-1)*self.kp*(self.target_angle-self.yaw)
                self.vel_pub.publish(command)
            else:
                command.angular.z=0.1
                self.vel_pub.publish(command)

            if abs(abs(self.target_angle)-abs(self.yaw))<0.01:
                
                command.angular.z=0.0
                self.vel_pub.publish(command)
                rospy.sleep(0.1)
                self.rotation_completed = True
                

            print("target={} current:{}".format(self.target_angle,self.yaw))
    
    def first_movement(self):
        if self.rotation_completed and self.values_imported and not self.first_movementf:
            rospy.loginfo("First Movement")
            
            if self.starting_point_left:
                command=Twist()
                command.linear.x=self.kp*(self.first_distance+self.linear_position_y)
            else:
                command=Twist()
                command.linear.x=0.1

            
            if self.init_flag:
                self.Final_point= self.linear_position_y-self.first_distance
                self.init_flag=False
            
        
            self.vel_pub.publish(command)
            if abs(abs(self.Final_point)-abs(self.linear_position_y))<0.01:
                command.linear.x=0.0
                self.vel_pub.publish(command)
                rospy.sleep(0.1)
                self.first_movementf = True
                
                

            print("target={} current:{}".format(self.first_distance,self.linear_position_y))

    def Rotate90(self):
        if self.rotation_completed and self.values_imported and self.first_movementf and not self.rotation90:
            if self.starting_point_left:
                command=Twist()
                command.angular.z=self.kp*(math.pi/2-self.yaw)
            else:
                command=Twist()
                command.angular.z=-0.1

            
            rospy.loginfo("Turning 90")
            
            self.vel_pub.publish(command)
            if abs(abs(self.yaw))<0.01:
                command.angular.z=0.0
                self.vel_pub.publish(command)
                rospy.sleep(0.1)
                self.rotation90 = True
            print("target={} current:{}".format(math.pi/2,self.yaw))


    def FinalPush(self,data):
        if self.rotation_completed and self.values_imported and self.first_movementf and self.rotation90:
            rospy.loginfo("For Final Push Hurrrraaaay!!!!")

            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
                return


            corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)

            if ids is not None:

                marker_center = np.mean(corners[0][0], axis=0)


                image_center = np.array([cv_image.shape[1] / 2, cv_image.shape[0] / 2])


                proportional_gain_angular = 0.0005
                angular_z = -proportional_gain_angular * (marker_center[0] - image_center[0])
                rospy.loginfo("angular_z %.6f", angular_z)

                proportional_gain_linear = 0.001
                linear_x = proportional_gain_linear * (image_center[1] - marker_center[1])  
                rospy.loginfo("linear_x %.6f", linear_x)

                twist_cmd = Twist()
                twist_cmd.angular.z = angular_z
                twist_cmd.linear.x = 0.1
                self.vel_pub.publish(twist_cmd)
            else:
            # If no marker is detected, stop the robot
                twist_cmd = Twist()
                twist_cmd.angular.z = 0
                twist_cmd.linear.x = 0
                self.vel_pub.publish(twist_cmd)
                rospy.loginfo("No marker detected. Stopping robot.")
                rospy.signal_shutdown("Final push completed.")


               


def main():
    try:
        centering=Odom()
        rate= rospy.Rate(10)
        #rospy.spin()
        
        while not rospy.is_shutdown():
            centering.rotate_robot_paralel()
            centering.first_movement()
            centering.Rotate90()
            #centering.FinalPush()


            rate.sleep()



    except rospy.ROSInterruptException:
        pass

if __name__=="__main__":
    main()
