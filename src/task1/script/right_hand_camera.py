#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float32MultiArray
def get_theta(src):
    #利用彩色图转化的二值图得到物快旋转角度
    # element = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
    # dst1 = cv.morphologyEx(src, cv.MORPH_OPEN, element)
    # dst1 = cv.morphologyEx(dst1, cv.MORPH_CLOSE, element)
    pixels = cv.countNonZero(src)
    dst1=src.copy()
    image, contours1, hierarchy = cv.findContours(dst1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    rect = cv.minAreaRect(contours1[0])
    box = cv.boxPoints(rect)
    x = box[0]
    y = box[1]
    z = box[2]
    w = box[3]
    l1 = np.sqrt(np.square(x[0] - y[0]) + np.square(x[1] - y[1]))
    l2 = np.sqrt(np.square(z[0] - y[0]) + np.square(z[1] - y[1]))
    if l1 > l2:
        p1 = x
        p2 = y
        l = l1
    else:
        p1 = y
        p2 = z
        l = l2
    if p1[0] == p2[0]:
        theta = 0
    elif p1[1] == p2[1]:
        theta = 90
    elif (p1[0] - p2[0]) * (p1[1] - p2[1]) > 0:
        theta = np.arccos(abs(p1[1] - p2[1]) / l) * 180 / np.pi
    elif (p1[0] - p2[0]) * (p1[1] - p2[1]) < 0:
        theta = 180 - np.arccos(abs(p1[1] - p2[1]) / l) * 180 / np.pi
    theta = round(theta)
    calc_pixels=l1*l2
    ratio=pixels/calc_pixels
    if(ratio<0.85):
        theta=90
    else:
        pass
    return theta
def near(img):
    copyimg = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    _, mask = cv.threshold(copyimg, 125, 255, cv.THRESH_BINARY)
    element = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
    dst = cv.morphologyEx(mask, cv.MORPH_OPEN, element)
    dst = cv.morphologyEx(dst, cv.MORPH_CLOSE, element)
    dst_copy = dst.copy()
    img3,contours3,hierarchy3 = cv.findContours(dst_copy,cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    draw0 = np.zeros([dst.shape[0], dst.shape[1]], dtype=np.uint8)
    cv.drawContours(draw0,contours3,-1,255,-1)
    x_center=0
    y_center=0
    area_list=[]
    if len(contours3) > 0:
        # max_index = 0
        # max_pixels = 0
        for contours_index in range(len(contours3)):
            single_masks = np.zeros((dst.shape[0],dst.shape[1])) 
            fill_image = cv.fillConvexPoly(single_masks, contours3[contours_index], 255)
            pixels = cv.countNonZero(fill_image)
            area_list.append([contours_index,pixels])
            # if pixels > max_pixels:
            #     max_pixels = pixels
            #     max_index = contours_index
        area_list.sort(key=lambda x:x[1],reverse=True)
    draw1=np.zeros([dst.shape[0], dst.shape[1]], dtype=np.uint8)
    cv.drawContours(draw1,contours3,area_list[0][0],255,-1)
    if(len(area_list)>1 and area_list[1][1]>100):
        cv.drawContours(draw1,contours3,area_list[1][0],255,-1)
    

    draw2 = draw1.copy()
    img2,contours2,hierarchy2 = cv.findContours(draw2,cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    #draw3 = np.zeros([dst.shape[0], dst.shape[1]], dtype=np.uint8)
    #cv.drawContours(draw3,contours2,-1,255,-1)
    lenght = []
    # # x_center,y_center = 0,0
    # threshould=40
    # valid_list=[]
    # for i in range(len(contours2)):
    #     area=cv.contourArea(contours2[i])
    #     if area>threshould:
    #         valid_list.append(i)
    for i in range(len(contours2)):
        # draw0 = np.zeros([dst.shape[0], dst.shape[1]], dtype=np.uint8)
        rect = cv.minAreaRect(contours2[i])
        box = cv.boxPoints(rect)
        x = box[0]
        y = box[1]
        z = box[2]
        w = box[3]
        x_center = int(x[0] / 4 + y[0] / 4 + z[0] / 4 + w[0] / 4)
        y_center = int(x[1] / 4 + y[1] / 4 + z[1] / 4 + w[1] / 4)
        l = np.sqrt(np.power(x_center-340,2)+np.power(y_center-190,2))
        lenght.append(l)
    if  len(lenght)==0:
        draw4 = np.zeros([dst.shape[0], dst.shape[1]], dtype=np.uint8)
        return 0,0,draw0,draw1,draw4
    else :

        arg = np.argmin(lenght)
        rect = cv.minAreaRect(contours2[arg])
        box = cv.boxPoints(rect)
        x = box[0]
        y = box[1]
        z = box[2]
        w = box[3]
        x_center = int(x[0] / 4 + y[0] / 4 + z[0] / 4 + w[0] / 4)
        y_center = int(x[1] / 4 + y[1] / 4 + z[1] / 4 + w[1] / 4)
        draw4 = np.zeros([dst.shape[0], dst.shape[1]], dtype=np.uint8)
        cv.drawContours(draw4, contours2, arg, 255, -1)
    return x_center,y_center,draw0,draw1,draw4


def callback0(imgmsg):
    pub_p = rospy.Publisher('right_hand_camera', Float32MultiArray, queue_size=10)
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(imgmsg)
    b = img[:,:,0]
    g = img[:,:,1]
    r = img[:,:,2]
    img = cv.merge([b,g,r])
    image = img.copy()

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    colorLow = np.array([0, 70, 150])
    colorHigh = np.array([40, 255, 255])
    #colorLow1 = np.array([0, 80, 46])
    #colorHigh1 = np.array([10, 255, 255])
    mask = cv.inRange(hsv, colorLow, colorHigh)
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (6, 6))
    mask= cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)#开操作 先腐蚀后膨胀
    mask= cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)#闭操作 先膨胀后腐蚀
    mask= cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    #x_center, y_center, draw = near(image)
    # x_center,y_center,draw0,draw1,draw4=near(image)
    # theta = get_theta(draw4)
    # if theta > 90:
    #     theta = theta - 180
    # print(x_center)
    # print(y_center)
    # print(theta)
    # img = fill_color(img,320,200)
    # element = cv.getStructuringElement(cv.MORPH_RECT, (9, 9))
    # dst = cv.morphologyEx(img, cv.MORPH_OPEN, element)
    # dst = cv.morphologyEx(dst, cv.MORPH_CLOSE, element)
    # index = np.where(dst == 255)
    # size = index[0].size
    # dst_copy = dst.copy()
    # x_center,y_center,theta= 0 ,0 ,0
    # if size<15000 and size>0:
    #     theta = get_theta(dst_copy)
    #     if theta > 90:
    #         theta = theta - 180
    #     img3,contours3,hierarchy3 = cv.findContours(dst_copy,cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    #     rect = cv.minAreaRect(contours3[0])
    #     box = cv.boxPoints(rect)
    #     x = box[0]
    #     y = box[1]
    #     z = box[2]
    #     w = box[3]
    #     x_center = int(x[0] / 4 + y[0] / 4 + z[0] / 4 + w[0] / 4)
    #     y_center = int(x[1] / 4 + y[1] / 4 + z[1] / 4 + w[1] / 4)
    #     # print(x_center,y_center,theta)
    # cv.circle(img, (x_center,y_center), 5, (0, 0, 255))
    # cv.circle(img, (352,173),5 , (255, 0, 0))
    #right_hand = Float32MultiArray(data = [x_center-340,y_center-190,theta] )
    # # rospy.loginfo(right_hand)
    #pub_p.publish(right_hand)
    #combine=np.hstack([draw0,draw1,draw4])
    # print("img")
    # print(img.shape)
    # print("draw")
    # print(draw.shape)
    #cv.imshow("combine_right", combine)
    cv.imshow("lipu",mask)
    cv.waitKey(2)



def listener():
    rospy.init_node('right_hand_image', anonymous=True)

    rospy.Subscriber("/cameras/right_hand_camera/image", Image, callback0)
    rospy.spin()



if __name__ == '__main__':
     listener()
