#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float32MultiArray
from PyQt5.Qt import QThread
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QDialog
from color_extract import Ui_Form
from std_msgs.msg import Int16
import sys
import threading
import tf
fx=603.149536#相机内参
fy=602.093201#相机内参
x_0=325.593140#相机内参
y_0=237.004852#相机内参
savePath="/home/wangzirui/HIT-DLR/src/task1/files/"
class Color_recognition:
    def __init__(self):
        rospy.init_node('left_hand_image', anonymous=True)
        self.hmax=0
        self.hmin=0
        self.smax=0
        self.smin=0
        self.vmax=0
        self.vmin=0
        self.sub = rospy.Subscriber("/cameras/left_hand_camera/image", Image, self.Callback)
        self.sub_one=rospy.Subscriber("Slider_one", Int16, self.Callback01)
        self.sub_two = rospy.Subscriber("Slider_two", Int16, self.Callback02)
        self.sub_three = rospy.Subscriber("Slider_three", Int16, self.Callback03)
        self.sub_four = rospy.Subscriber("Slider_four", Int16, self.Callback04)
        self.sub_five = rospy.Subscriber("Slider_five", Int16, self.Callback05)
        self.sub_six = rospy.Subscriber("Slider_six", Int16, self.Callback06)
        self.sub_depth=rospy.Subscriber("/camera/aligned_depth_to_color/image",Image,self.Callback07)
        self.realsense_traj=np.empty(shape=[0,3])
        self.baxter_traj=np.empty(shape=[0,3])
        self.realsense_onetraj=np.zeros(3)
        self.baxter_onetraj=np.zeros(3)
        self.isadd=True
        self.tf_listener=tf.TransformListener()
        #self.rate=rospy.Rate(50)
        rospy.loginfo("Create the class!")
    def Callback01(self,msg):
        self.hmax=msg.data
    def Callback02(self,msg):
        self.hmin=msg.data
    def Callback03(self,msg):
        self.smax=msg.data
    def Callback04(self,msg):
        self.smin=msg.data
    def Callback05(self,msg):
        self.vmax=msg.data
    def Callback06(self,msg):
        self.vmin=msg.data
    def Callback07(self,imgmsg):
        bridge = CvBridge()
        self.depth_img = bridge.imgmsg_to_cv2(imgmsg)
        #print(img)
        #print(img.shape)
    def Callback(self,imgmsg):

        bridge = CvBridge()

        img = bridge.imgmsg_to_cv2(imgmsg)
        r = img[:, :, 0]
        g = img[:, :, 1]
        b = img[:, :, 2]
        img = cv.merge([b, g, r])
        image = img.copy()
        #print(image.shape)
        lab_image = cv.cvtColor(image, cv.COLOR_RGB2LAB)
       
        # colorLow = np.array([20, 0, 50])
        # colorHigh = np.array([70, 80, 220])
        colorLow = np.array([self.hmin, self.smin, self.vmin])
        # print(self.hmax)
        colorHigh = np.array([self.hmax, self.smax, self.vmax])
        # colorLow = np.array([194, 105, 149])
        # colorHigh = np.array([255, 161, 185])
        mask_test = cv.inRange(lab_image, colorLow, colorHigh)
        # kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
        # ret1 = cv.morphologyEx(mask_test, cv.MORPH_CLOSE, kernel, iterations=2)
        # ret1 = cv.morphologyEx(mask_test, cv.MORPH_OPEN,kernel, iterations=2)
        # temp = np.ones(mask_test.shape, np.uint8) * 0
        # _,contours, hierarchy = cv.findContours(ret1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        # if len(contours) > 0:
        #     max_index = 0
        #     max_pixels = 0
        #     for contours_index in range(len(contours)):
        #         single_masks = np.zeros((640,480)) 
        #         fill_image = cv.fillConvexPoly(single_masks, contours[contours_index], 255)
        #         pixels = cv.countNonZero(fill_image)

        #         if pixels > max_pixels:
        #             max_pixels = pixels
        #             max_index = contours_index
            
        #     # 画出轮廓：temp是白色幕布，contours是轮廓，-1表示全画，然后是颜色，厚度
        #     cv.drawContours(temp, contours, max_index, 255, thickness=-1)
        #     point_list=np.where(temp==255)
        #     x_sum=0
        #     y_sum=0
        #     count=0
        #     #print(len(point_list[0]))
        #     #print(len(point_list[1]))
        #     for i in range(len(point_list[0])):
        #         x_sum=x_sum+point_list[1][i]
        #         y_sum=y_sum+point_list[0][i]
        #         count=count+1
        #     x_center=np.int(x_sum)/count
        #     y_center=np.int(y_sum)/count
        #     cv.circle(temp,(x_center,y_center),5,(0,0,0),3)
        #     camera_z=np.float(self.depth_img[y_center][x_center])/1000
        #     x_div_z=(x_center-x_0)/fx
        #     y_div_z=(y_center-y_0)/fy
        #     camera_x=camera_z*x_div_z
        #     camera_y=camera_z*y_div_z
        #     if(camera_z!=0):
        #         self.realsense_onetraj[0]=camera_x
        #         self.realsense_onetraj[1]=camera_y
        #         self.realsense_onetraj[2]=camera_z
        #         self.BaxterTraj()
        #     else:
        #         pass  
        # else:
        #     print("G")
        cv.imshow("img",mask_test)
        k=cv.waitKey(1)
        if k==27:
            self.savedata()
            rospy.signal_shutdown("Task is over!")


    # def process(self):
    #     thread.start_new_thread(self.showUI())
    #     #thread.start_new_thread(self.showPic())
    ##3.通过tf传回机械臂相对于机器人的位置
    def getTfFromMatrix(self,matrix):
        scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(matrix)
        return trans, tf.transformations.quaternion_from_euler(*angles), angles

# lookup tf transform between two frames
    def lookupTransform(self,tf_listener, target, source):
        tf_listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(4.0))
        
        trans, rot = tf_listener.lookupTransform(target, source, rospy.Time())
        euler = tf.transformations.euler_from_quaternion(rot)
        source_target = tf.transformations.compose_matrix(translate = trans, angles = euler)
        
        return source_target
    def BaxterTraj(self):
        base_marker = self.lookupTransform(self.tf_listener, '/left_gripper', '/base')
        trans_baxter, rot, rot_euler = self.getTfFromMatrix(np.linalg.inv(base_marker))
        self.baxter_onetraj=np.asarray(trans_baxter)
        self.realsense_traj=np.vstack([self.realsense_traj,self.realsense_onetraj])
        self.baxter_traj=np.vstack([self.baxter_traj,self.baxter_onetraj])
    def savedata(self):
        print ('write tracking data to files')
        np.savetxt('%sbaxter_trajectory' % (savePath), self.baxter_traj, delimiter=',', fmt='%.4f')
        np.savetxt('%srealsense_trajectory' % (savePath), self.realsense_traj, delimiter=',', fmt='%.4f')


if __name__=="__main__":
   color=Color_recognition()
   rospy.spin()