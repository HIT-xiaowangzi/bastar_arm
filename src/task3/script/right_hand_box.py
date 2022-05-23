#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
i=0
def Callback(imgmsg):
    global i
    bridge=CvBridge()
    image=bridge.imgmsg_to_cv2(imgmsg)
    r=image[:,:,0]
    g=image[:,:,1]
    b=image[:,:,2]
    img=cv2.merge([r,g,b])
    image=img.copy()
    cv2.imshow("zijixie",image)
    k=cv2.waitKey(1)
    if(k==27):
        cv2.imwrite("/home/wangzirui/桌面/4.17低分辨率补充/picture"+str(i)+".jpg",image)
        print("picture"+str(i)+".jpg saved successfully!")
        i=i+1
def main():
    rospy.init_node("g")
    sub_p=rospy.Subscriber("/cameras/right_hand_camera/image",Image,Callback,queue_size=1,buff_size=100000)
    rospy.Rate(50)
if __name__=="__main__":
    main()
    rospy.spin()