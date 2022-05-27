#!/usr/bin/python
# -*- coding: utf-8 -*-

#from pickletools import uint8
import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
#from torch import double
#from torch import int16
first=True
width_pad=80
height_pad=40
def getCenter(height,width,contours,index):
    draw_temp=np.zeros([height,width],dtype=np.uint8)
    cv.drawContours(draw_temp,contours,index,255,-1)
    point_list=np.where(draw_temp==255)
    x_sum=0
    y_sum=0
    count=0
    #print(len(point_list[0]))
    #print(len(point_list[1]))
    for i in range(len(point_list[0])):
        x_sum=x_sum+point_list[1][i]
        y_sum=y_sum+point_list[0][i]
        count=count+1
    x_center=np.float(x_sum)/count
    y_center=np.float(y_sum)/count
    print(x_center,y_center)
    return x_center,y_center
def get_theta(src):
    #利用彩色图转化的二值图得到物快旋转角度
    element = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
    dst1 =cv.dilate(src,element,iterations=2)
    dst1 = cv.morphologyEx(dst1, cv.MORPH_CLOSE, element)
    # dst1 = cv.morphologyEx(src, cv., element)
    # dst1 = cv.morphologyEx(dst1, cv.MORPH_CLOSE, element)
    pixels = cv.countNonZero(src)
    dst1=src.copy()
    image, contours1, hierarchy = cv.findContours(dst1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    print(len(contours1))
    if(len(contours1)==0 or len(contours1)>2):
        return 90
    elif (len(contours1)==1):
        rect = cv.minAreaRect(contours1[0])
        box = cv.boxPoints(rect)
        x = box[0]
        y = box[1]
        z = box[2]
        w = box[3]
        #cv.rectangle(dst1,x,z,True,255,3)
        #cv.imshow("ceshi",dst1)
        #print(x,y,z,w)
        #cv.waitKey(1)
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
        if(theta>90):
            theta=theta-180
        return theta
    else:
        x_center_one,y_center_one=getCenter(dst1.shape[0],dst1.shape[1],contours1,0)
        x_center_two,y_center_two=getCenter(dst1.shape[0],dst1.shape[1],contours1,1)
        if(x_center_one==x_center_two):
            theta=0
        elif(y_center_one==y_center_two):
            theta=90
        else:
            theta=np.arctan(-(y_center_one-y_center_two)/(x_center_one-x_center_two))
            print("Enter!")
            theta=theta*180/np.pi
            theta=round(theta)
            if(0<theta and theta<90):
                theta=theta-90
            else:
                theta=theta+90
        return theta

    # rect = cv.minAreaRect(contours1[0])
    # box = cv.boxPoints(rect)
    # x = box[0]
    # y = box[1]
    # z = box[2]
    # w = box[3]
    # #cv.rectangle(dst1,x,z,True,255,3)
    # #cv.imshow("ceshi",dst1)
    # #print(x,y,z,w)
    # cv.waitKey(1)
    # l1 = np.sqrt(np.square(x[0] - y[0]) + np.square(x[1] - y[1]))
    # l2 = np.sqrt(np.square(z[0] - y[0]) + np.square(z[1] - y[1]))
    # if(x[0]==y[0]):
    #     theta_1=90
    # else:
    #     theta_1=np.arctan(-(x[1]-y[1])/(x[0]-y[0]))
    # if (z[0]==y[0]):
    #     theta_2=90
    # else:
    #     theta_2=np.arctan(-(z[1]-y[1])/(z[0]-y[0]))
    # if(abs(theta_1)>abs(theta_2)):
    #     theta=theta_1
    # else:
    #     theta=theta_2
    # theta=theta*180/np.pi
    # theta=round(theta)
    # print(l1)
    # print(l2)
   
    # calc_pixels=l1*l2
    # ratio=pixels/calc_pixels
    # print(calc_pixels)
    # print(ratio)
    # if(pixels>3000 and ratio<0.85):
    #     if(abs(theta)<45):
    #         if(0<theta and theta<=90):
    #             theta=theta-90
    #         else:
    #             theta=theta+90
    #     else:
    #         pass
    # elif(pixels<=3000 and ratio<0.85):
    #     theta=90
    # else:
    #     pass
    # if(theta>90):
    #     theta=90
    # return theta
def near(img):
    global width_pad
    global height_pad
    mask_table=np.zeros([img.shape[0],img.shape[1]], dtype=np.uint8)
    print(img.shape[0])
    mask_table[height_pad:img.shape[0]-height_pad,width_pad:img.shape[1]-width_pad]=255#外框涂黑
    copyimg = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    _, mask = cv.threshold(copyimg, 125, 255, cv.THRESH_BINARY)
    mask=cv.bitwise_and(mask,mask,mask=mask_table)
    element = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
    dst = cv.morphologyEx(mask, cv.MORPH_OPEN, element)
    dst = cv.morphologyEx(dst, cv.MORPH_CLOSE, element)
    dst_copy = dst.copy()
    
    img3,contours3,hierarchy3 = cv.findContours(dst_copy,cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    draw0 = np.zeros([dst.shape[0], dst.shape[1]], dtype=np.uint8)
    draw1=np.zeros([dst.shape[0], dst.shape[1]], dtype=np.uint8)
    cv.drawContours(draw0,contours3,-1,255,-1)
    x_center=0
    y_center=0
    valid_list=[]
    area_list=[]
    if len(contours3) > 0:
        # max_index = 0
        # max_pixels = 0
        # for contour_index in range(len(contours3)):
        #     draw_temp=np.zeros([dst.shape[0],dst.shape[1]],dtype=np.uint8)
        #     cv.drawContours(draw_temp,contours3,contour_index,255,-1)
        #     points=np.where(draw_temp==255)
        #     print(points[0])
        #     if(len(np.where(points[0]==0))<2 and len(np.where(points[0]==dst.shape[0]-1))<2
        #     and len(np.where(points[1]==0))<2 and len(np.where(points[1]==dst.shape[1]-1))<2):
                
        #         valid_list.append(contour_index)
        #     print(valid_list)
        for contours_index in range(len(contours3)):
            single_masks = np.zeros((dst.shape[0],dst.shape[1])) 
            fill_image = cv.fillConvexPoly(single_masks, contours3[contours_index], 255)
            pixels = cv.countNonZero(fill_image)
            area_list.append([contours_index,pixels])
            # if pixels > max_pixels:
            #     max_pixels = pixels
            #     max_index = contours_index
        area_list.sort(key=lambda x:x[1],reverse=True)
        cv.drawContours(draw1,contours3,area_list[0][0],255,-1)
    if(len(area_list)>1 and area_list[1][1]>300):
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
        l = np.sqrt(np.power(x_center-328,2)+np.power(y_center-200,2))
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
    global first
    if(first==False):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg)
        b = img[:,:,0]
        g = img[:,:,1]
        r = img[:,:,2]
        img = cv.merge([b,g,r])
        image = img.copy()
        #x_center, y_center, draw = near(image)
        x_center,y_center,draw0,draw1,draw4=near(image)
        theta = get_theta(draw1)
        
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
        right_hand = Float32MultiArray(data = [x_center-328,y_center-200,theta] )
        # # rospy.loginfo(right_hand)
        pub_p.publish(right_hand)
        print(theta)
        combine=np.hstack([draw0,draw1,draw4])
        # print("img")
        # print(img.shape)
        # print("draw")
        # print(draw.shape)
        cv.imshow("combine_left", combine)
        k=cv.waitKey(1)
        if k==27:
            filename="/home/wangzirui/学姐图片/depth.jpg"
            cv.imwrite(filename,combine)

def block_callback(msg):
    global first
    if(msg.data!="open"):
        print("Wait!")
    else:
        first=False
        print("Start!")
    



if __name__ == '__main__':
    rospy.init_node('left_hand_image', anonymous=True)
    rospy.Subscriber("block_switch", String, block_callback, queue_size=2)
    pub_p = rospy.Publisher('left_hand_camera', Float32MultiArray, queue_size=10)
    rospy.Subscriber("/cameras/left_hand_camera/image", Image, callback0)
    #rospy.Subscriber("/camera/color/image_raw",Image,callback0)
    rospy.spin()
