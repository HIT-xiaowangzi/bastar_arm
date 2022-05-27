#!/usr/bin/python 
# -*- coding: utf-8 -*-


import rospy
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2,Image
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
from cv_bridge import CvBridge
import tf
import yaml
savePath="/home/wangzirui/HIT-DLR/src/task1/files/"
class Calibration:
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        self.sub1=rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.getPointcloud)
        self.sub2=rospy.Subscriber("/camera/color/image_raw",Image,self.getImg)
        self.sub3=rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",Image,self.getDepthImg)
        self.realsense_traj=np.empty(shape=[0,3])
        self.baxter_traj=np.empty(shape=[0,3])
        self.realsense_onetraj=np.zeros(3)
        self.baxter_onetraj=np.zeros(3)
        self.isadd=True
        self.tf_listener=tf.TransformListener()
        #self.rate=rospy.Rate(50)
        rospy.loginfo("Create the class!")
    ##1.接受到点云图像
    def getPointcloud(self,pointcloud):
        pc = ros_numpy.numpify(pointcloud)
        height = pc.shape[0]
        width = pc.shape[1]
        print(height)
        print(width)
        self.np_points = np.zeros((height * width, 3), dtype=np.float32)
        self.np_points[:, 0] = np.resize(pc['x'], height * width)
        self.np_points[:, 1] = np.resize(pc['y'], height * width)
        self.np_points[:, 2] = np.resize(pc['z'], height * width)
        print(self.np_points)
        #self.rate.sleep()
##2.通过opencv回传物块中心点
    def getDepthImg(self,imgmsg):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg)
        depth_img=img.copy()
        depth_img=depth_img.astype(np.float32)
        depth_img=np.clip(depth_img,0,2550)/10
        depth_img[depth_img==0]=255
        depth_img=depth_img.astype(np.uint8)
        _,depth_img=cv2.threshold(depth_img,90,255,cv2.THRESH_BINARY_INV)
        print(depth_img)
        #depth_img, contours, hierarchy = cv2.findContours(depth_img,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        #depth_img=cv2.drawContours(depth_img,contours,-1,255,-1)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (6, 6))
        binary = cv2.morphologyEx(depth_img, cv2.MORPH_OPEN, kernel)#开操作 先腐蚀后膨胀
        #binary = cv2.morphologyEx(depth_img, cv2.MORPH_OPEN, kernel)#开操作 先腐蚀后膨胀
        #binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)#闭操作 先膨胀后腐蚀
        self.binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)#闭操作 先膨胀后腐蚀
        #cv2.imshow("depth_image",binary)
        #cv2.waitKey(1)
    def getImg(self,imgmsg):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg)
        r = img[:,:,0]
        g = img[:,:,1]
        b = img[:,:,2]
        self.img = cv2.merge([b,g,r])##ros图像转成cv
        self.Cameratraj(self.img,self.np_points)
        self.BaxterTraj()
        #rospy.on_shutdown(self.savedata)
    def Cameratraj(self,img,np_points):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        colorLow = np.array([78, 43, 46])
        colorHigh = np.array([99, 255, 255])
        #colorLow1 = np.array([0, 80, 46])
        #colorHigh1 = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, colorLow, colorHigh)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (6, 6))
        mask= cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)#开操作 先腐蚀后膨胀
        mask= cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)#闭操作 先膨胀后腐蚀
        mask= cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask=cv2.bitwise_and(self.binary,mask)
        #mask1 = cv2.inRange(hsv, colorLow1, colorHigh1)
        #mask = cv2.bitwise_or(mask,mask1)
        #hsv=cv2.bitwise_and(hsv,mask)
        #print(mask)
        #cv2.imshow("blue",mask)
        #cv2.waitKey(1)
        _,contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) > 0:
            max_index = 0
            max_pixels = 0
            for contours_index in range(len(contours)):
                single_masks = np.zeros((640,480)) 
                fill_image = cv2.fillConvexPoly(single_masks, contours[contours_index], 255)
                pixels = cv2.countNonZero(fill_image)

                if pixels > max_pixels:
                    max_pixels = pixels
                    max_index = contours_index
            temp = np.ones(mask.shape, np.uint8) * 0
            # 画出轮廓：temp是白色幕布，contours是轮廓，-1表示全画，然后是颜色，厚度
            cv2.drawContours(temp, contours, max_index, 255, thickness=-1)  ###temp是轮廓最大的
            # dilate_element = cv2.getStructuringElement(cv2.MORPH_RECT,(4,40))
            # dilate_temp = cv2.dilate(temp, dilate_element)
            target_contour = contours[max_index]
            target_contour = np.squeeze(target_contour)
            center = np.mean(target_contour, axis=0)  
            center=center.astype(np.uint16)  ###轮廓最大的中心点
            #print(center.shape)
            cv2.circle(temp,(center[0],center[1]),5,(0,0,0),3)
            combine=np.hstack([mask,temp])
            cv2.imshow("combine",combine)
            cv2.waitKey(1)
            #cv2.imshow("result",temp)
            #cv2.waitKey(1)
            target_point=center[1]*640+center[0]
            if(np_points[target_point][0]!=np.nan and np_points[target_point][1]!=np.nan and np_points[target_point][2]!=np.nan ):
                self.realsense_onetraj[0]=np_points[target_point][0]
                self.realsense_onetraj[1]=np_points[target_point][1]
                self.realsense_onetraj[2]=np_points[target_point][2]
                self.isadd=True
            else:
                self.isadd=False
                rospy.loginfo("Pointcloud data is invalid!")
        else:
            rospy.loginfo("Can't recognize the card!")
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
        if(self.isadd):
            self.realsense_traj=np.vstack([self.realsense_traj,self.realsense_onetraj])
            self.baxter_traj=np.vstack([self.baxter_traj,self.baxter_onetraj])
        else:
            pass
    def savedata(self):
        print ('write tracking data to files')
        np.savetxt('%sbaxter_trajectory' % (savePath), self.baxter_traj, delimiter=',', fmt='%.4f')
        np.savetxt('%srealsense_trajectory' % (savePath), self.realsense_traj, delimiter=',', fmt='%.4f')
        
    
def main():
    calibration=Calibration()
    rospy.spin()
if __name__ == "__main__":

    main()
