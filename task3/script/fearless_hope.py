#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
from copy import deepcopy
import numpy as np
import numpy as np
import numpy.linalg as LA
#Cite as:An algorithm previously used in radio astronomy [1].大佬喝茶！
def PJcurvature(x,y):#计算曲线的曲率半径
    """
    input  : the coordinate of the three point
    output : the curvature and norm direction
    refer to https://github.com/Pjer-zhang/PJCurvature for detail
    """
    t_a = LA.norm([x[1]-x[0],y[1]-y[0]])
    t_b = LA.norm([x[2]-x[1],y[2]-y[1]])
    
    M = np.array([
        [1, -t_a, t_a**2],
        [1, 0,    0     ],
        [1,  t_b, t_b**2]
    ])

    a = np.matmul(LA.inv(M),x)
    b = np.matmul(LA.inv(M),y)

    kappa = 2*(a[2]*b[1]-b[2]*a[1])/(a[1]**2.+b[1]**2.)**(1.5)
    return kappa, [b[1],-a[1]]/np.sqrt(a[1]**2.+b[1]**2.)
    #kappa:曲率 norm:矢径的方向余弦
def callback0(imgmsg):
    bridge=CvBridge()
    img = bridge.imgmsg_to_cv2(imgmsg)
    r = img[:,:,0]
    g = img[:,:,1]
    b = img[:,:,2]
    img = cv.merge([b,g,r])
    image = img.copy()
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    ret, binary = cv.threshold(gray, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
    k = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
    # 开操作
    binary = cv.morphologyEx(binary, cv.MORPH_OPEN, k)
    area_list=[]
    # 轮廓发现
    _,contours, hierarchy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
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
        for contours_index in range(len(contours)):
            single_masks = np.zeros((binary.shape[0],binary.shape[1])) 
            fill_image = cv.fillConvexPoly(single_masks, contours[contours_index], 255)
            pixels = cv.countNonZero(fill_image)
            area_list.append([contours_index,pixels])
            # if pixels > max_pixels:
            #     max_pixels = pixels
            #     max_index = contours_index
        area_list.sort(key=lambda x:x[1],reverse=True)
    draw1=np.zeros([binary.shape[0], binary.shape[1]], dtype=np.uint8)
    draw1 = cv.morphologyEx(draw1, cv.MORPH_OPEN, k)
    draw1 = cv.morphologyEx(draw1, cv.MORPH_OPEN, k)
    cv.drawContours(draw1,contours,area_list[0][0],255,-1)
    cv.drawContours(image,contours,area_list[0][0],(0,0,255),3)
    # if(len(area_list)>1 and area_list[1][1]>300):
    #     cv.drawContours(draw1,contours,area_list[1][0],255,-1)
    _,contours3,hierarchy2=cv.findContours(draw1,cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    # print(contours2[0][0][0][0])
    # print(contours2[0][0][0][1])
    kappa=[]#曲率
    norm=[]#指向曲率中心的矢径
    point=[]
    for j in range(len(contours3[0])):
        if(j%5==0):
            point.append((contours3[0][j][0][0],contours3[0][j][0][1]))

    length=len(point)
    if(length>3):#多于3个点才能求曲率啊
        for i in range(length):
            x=[]#存第i-1个点，第i个点，第i+1个点的坐标
            y=[]
            if i==0:
                x.append(point[length-1][0])
                x.append(point[0][0])
                x.append(point[1][0])
                y.append(480-point[length-1][1])
                y.append(480-point[0][1])
                y.append(480-point[1][1])
            elif i==length-1:
                x.append(point[length-2][0])
                x.append(point[length-1][0])
                x.append(point[0][0])
                y.append(480-point[length-2][1])
                y.append(480-point[length-1][1])
                y.append(480-point[0][1])
            else:
                x.append(point[i-1][0])
                x.append(point[i][0])
                x.append(point[i+1][0])
                y.append(480-point[i-1][1])
                y.append(480-point[i][1])
                y.append(480-point[i+1][1])
            kappa_temp,norm_temp=PJcurvature(x,y)
            kappa.append(kappa_temp)
            norm.append(norm_temp)
    #print(kappa)
    #print(norm)
    kappa_copy=[]
    for i in kappa:
        kappa_copy.append(abs(i))
    kappa_copy.sort()
    ratio=0.85
    index=int(length*ratio)
    kappa_threshould=kappa_copy[index]
    for i in range(length):
        if(abs(kappa[i])>kappa_threshould):
            # cv.arrowedLine(image,
            #         pt1=(int(contours2[0][i][0][0]),int(contours2[0][i][0][1])),
            #         pt2=(int(contours2[0][i][0][0]+norm[i][0]/kappa[i]*3),int(contours2[0][i][0][1]+norm[i][1]/kappa[i]*3)),
            #         color=(255, 255, 0),
            #         thickness=2,
            #         line_type=cv.LINE_8,
            #         shift=0,
            #         tipLength=0.1)
            cv.circle(image,(point[i][0],point[i][1]),2,(255,0,0),2)

    # 在原图上绘制轮廓，以方便和凸包对比，发现凸缺陷
    # cv.drawContours(draw1, contours, -1, (0, 225, 0), 3)
    # for c in range(len(contours)):
    #     # 是否为凸包
    #     ret = cv.isContourConvex(contours[c])
    #     # 凸缺陷
    #     # 凸包检测，returnPoints为false的是返回与凸包点对应的轮廓上的点对应的index
    #     hull = cv.convexHull(contours[c], returnPoints=False)
    #     defects = cv.convexityDefects(contours[c], hull)
    #     print('defects', defects)
    #     for j in range(defects.shape[0]):
    #         s, e, f, d = defects[j, 0]
    #         start = tuple(contours[c][s][0])
    #         end = tuple(contours[c][e][0])
    #         far = tuple(contours[c][f][0])
    #         # 用红色连接凸缺陷的起始点和终止点
    #         cv.line(image, start, end, (0, 0, 225), 2)
    #         # 用蓝色最远点画一个圆圈
    #         cv.circle(image, far, 5, (225, 0, 0), -1)
    cv.imshow("jiayou",image)
    cv.waitKey(1)
def listener():
    rospy.init_node('left_hand_image', anonymous=True)
    rospy.Subscriber("/cameras/left_hand_camera/image", Image, callback0)
    rospy.spin()

if __name__ == '__main__':
     listener()