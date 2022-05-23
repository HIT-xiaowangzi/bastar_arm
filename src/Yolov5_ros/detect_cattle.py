#!/usr/bin/env python
# -*- coding: utf-8 -*-

from copy import deepcopy
import roslib
import rospy
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
IMAGE_WIDTH=640
IMAGE_HEIGHT=480

import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')




import os
import time
import cv2
import torch
from numpy import random
import torch.backends.cudnn as cudnn
import numpy as np
import math
from models.experimental import attempt_load
from utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords,
    xyxy2xywh, plot_one_box, strip_optimizer, set_logging)
from utils.torch_utils import select_device, load_classifier, time_synchronized

from matplotlib import pyplot as plt
from task2.msg import MyList,MyListArray

ros_image=0
mask_global=0
class Point:
    def __init__(self,cattle:MyList,bottom:MyList):
        self.cattle=cattle
        self.bottom=bottom
        self.flag=True
        self.radius=110
        self.theta=math.atan2(cattle.y-bottom.y,bottom.x-cattle.x)
        self.CalcPoint()
    def CalcPoint(self):
        self.left=(int(self.cattle.x+self.radius*math.cos(-self.theta+math.pi/2)),int(self.cattle.y+self.radius*math.sin(self.theta+math.pi/2)))
        self.right=(int(self.cattle.x+self.radius*math.cos(-self.theta-math.pi/2)),int(self.cattle.y+self.radius*math.sin(self.theta-math.pi/2)))
        if mask_global[self.left[1]][self.left[0]]==0 and mask_global[self.right[1]][self.right[0]]==0:
            self.flag=True
        else:
            self.flag=False
        cv2.drawMarker(mask_global,self.left,(255,255,255),markerType=2)
        cv2.drawMarker(mask_global,self.right,(255,255,255),markerType=2)
def dist(a:MyList,b:MyList):
    return math.pow((a.x-b.x),2)+math.pow((a.y-b.y),2)
def MinInd(cattle:MyList,Wrist_Bottom_copy:MyListArray):
    size=len(Wrist_Bottom_copy.MyLists)
    min_ind=0
    min_val=dist(cattle,Wrist_Bottom_copy.MyLists[0])
    for i in range(1,size):
        val=dist(cattle,Wrist_Bottom_copy.MyLists[1])
        if val<min_val:
            min_val=val
            min_ind=i
    return min_ind
def MaxConf(cattle:MyList,Wrist_Bottom_copy:MyListArray):
    size=len(Wrist_Bottom_copy.MyLists)
    max_ind=0
    max_val=Wrist_Bottom_copy.MyLists[0].confidence
    for i in range(1,size):
        val=Wrist_Bottom_copy.MyLists[i].confidence
        if val>max_val:
            max_val=val
            max_ind=i
    return max_ind
def FindBottom(cattle:MyList,Wrist_Bottom:MyListArray):
    flag=True
    Bottom=MyList()
    Wrist_Bottom_vaild=MyListArray()
    size=len(Wrist_Bottom.MyLists)
    x_border=(cattle.x-cattle.w/2,cattle.x+cattle.w/2)
    y_border=(cattle.y-cattle.h/2,cattle.y+cattle.h/2)
    #把不在2框内的7删掉
    for i in range(size):
        conf=Wrist_Bottom.MyLists[i].confidence
        if(conf<0.6):
            continue
        if((x_border[0]<Wrist_Bottom.MyLists[i].x and Wrist_Bottom.MyLists[i].x<x_border[1] \
        and y_border[0]<Wrist_Bottom.MyLists[i].y and Wrist_Bottom.MyLists[i].y<y_border[1])):
            Wrist_Bottom_vaild.MyLists.append(Wrist_Bottom.MyLists[i])
    new_size=len(Wrist_Bottom_vaild.MyLists)
    if(new_size==0):
        flag=False
    else:
        for j in range(new_size):
            ind=MaxConf(cattle,Wrist_Bottom_vaild)
            Bottom=Wrist_Bottom_vaild.MyLists[ind]
    return flag,Bottom
def preprocess(Wrist_Cattle:MyListArray,Wrist_Bottom:MyListArray):
    #经过处理，返回可用的牛头，牛腚及是否可被抓取
    Wrist_FinalCattle=MyListArray()
    Wrist_FinalBottom=MyListArray()
    size=len(Wrist_Cattle.MyLists)
    print("size:[%ld]",size)
    #1.找到所有符合标准的全牛
    for i in range(size):
        #conf=Wrist_Cattle.MyLists[i].confidence
        #if(conf<0.8):
            #continue
        min_num=min(Wrist_Cattle.MyLists[i].w,Wrist_Cattle.MyLists[i].h)
        max_num=max(Wrist_Cattle.MyLists[i].w,Wrist_Cattle.MyLists[i].h)
        if(min_num<75 or max_num<150):
            continue
        Wrist_FinalCattle.MyLists.append(Wrist_Cattle.MyLists[i])
        #2.对于一只全牛，求出他的屁股的位置，没有屁股直接扔掉
        flag,bottom=FindBottom(Wrist_Cattle.MyLists[i],Wrist_Bottom)
        if(flag==True):
            #3.判断这头牛是在上面还是在下面
            point=Point(Wrist_Cattle.MyLists[i],bottom)
            if(point.flag==True):
                Wrist_Cattle.MyLists[i].confidence=1#用置信度表示flag
            else:
                Wrist_Cattle.MyLists[i].confidence=0
            Wrist_FinalBottom.MyLists.append(bottom)
    return Wrist_FinalCattle,Wrist_FinalBottom
def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True):
    # Resize image to a 32-pixel-multiple rectangle https://github.com/ultralytics/yolov3/issues/232
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)
    
    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, 32), np.mod(dh, 32)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return img, ratio, (dw, dh)
def loadimg(img):  # 接受opencv图片
    img_size=640
    cap=None
    path=None
    img0 = img
    img = letterbox(img0, new_shape=img_size)[0]
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    img = np.ascontiguousarray(img)
    return path, img, img0, cap
# src=cv2.imread('biye.jpg')
def detect(img):
    global mask_global
    Wrist_Cattle=MyListArray()
    Wrist_Bottom=MyListArray()
    Wrist_CattleList=MyList()
    Wrist_BottomList=MyList()
    #Wrist_CattleList.clear()
    #Wrist_BottomList.clear()
    time1 = time.time()

    global ros_image
    cudnn.benchmark = True
    dataset = loadimg(img)
    # print(dataset[3])
    #plt.imshow(dataset[2][:, :, ::-1])
    names = model.module.names if hasattr(model, 'module') else model.names
    #colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]
    #colors=[[0,255,0]]
    augment = 'store_true'
    conf_thres = 0.3
    iou_thres = 0.45
    classes = (0,1,2)
    agnostic_nms = 'store_true'
    img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
    _ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once
    path = dataset[0]
    img = dataset[1]
    im0s = dataset[2]
    vid_cap = dataset[3]
    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0

    time2 = time.time()
    if img.ndimension() == 3:
        img = img.unsqueeze(0)
    print(img.shape)
    # Inference
    pred = model(img, augment=augment)[0]
    # Apply NMS
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes=classes, agnostic=agnostic_nms)

    view_img = 1
    save_txt = 1
    save_conf = 'store_true'
    time3 = time.time()

    for i, det in enumerate(pred):  # detections per image
        p, s, im0 = path, '', im0s
        s += '%gx%g ' % img.shape[2:]  # print string
        gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        if det is not None:
            #print(det)
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
            # Print results
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
                s += '%g %ss, ' % (n, names[int(c)])  # add to string
                # Write results
            for *xyxy, conf, cls in reversed(det):
                xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()
                c=int(cls)
                if save_txt:  # Write to file
                    #xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    line = (cls, conf, *xywh) if save_conf else (cls, *xywh)  # label format
                if view_img:  # Add bbox to image
                    label = '%s %.2f' % (names[int(cls)], conf)
                    plot_one_box(xyxy, im0, label=label, color=[0,255,0], line_thickness=3)
                if c==0:#整头牛
                    Wrist_CattleList.x=int(xywh[0]*640)   
                    Wrist_CattleList.y=int(xywh[1]*400)
                    Wrist_CattleList.w=int(xywh[2]*640)
                    Wrist_CattleList.h=int(xywh[3]*400)
                    Wrist_CattleList.class_=c+2
                    Wrist_CattleList.confidence=conf
                    Wrist_Cattle.MyLists.append(deepcopy(Wrist_CattleList))
                    center=(int(xywh[0]*640),int(xywh[1]*400))
                    cv2.circle(im0,center,3,(255,0,0),2)
                    cv2.circle(mask_global,center,3,(0,0,0),2)
                else:
                    Wrist_BottomList.x=int(xywh[0]*640)     
                    Wrist_BottomList.y=int(xywh[1]*400)  
                    Wrist_BottomList.w=int(xywh[2]*640)
                    Wrist_BottomList.h=int(xywh[3]*400)   
                    Wrist_BottomList.class_=c+6
                    Wrist_BottomList.confidence=conf
                    Wrist_Bottom.MyLists.append(deepcopy(Wrist_BottomList))  
                    center_bottom=(int(xywh[0]*640),int(xywh[1]*400))
                    cv2.drawMarker(mask_global,center_bottom,(0,0,0),markerType=0) 
                # theta=math.atan2(center[1]-center_bottom[1],center_bottom[0]-center[0])
                # radius=45
                # point_one=(int(center[0]+radius*math.cos(theta+math.pi/2)),int(center[1]-radius*math.sin(theta+math.pi/2)))
                # point_two=(int(center[0]+radius*math.cos(theta-math.pi/2)),int(center[1]-radius*math.sin(theta-math.pi/2)))
                # cv2.drawMarker(mask_global,point_one,(255,255,255),markerType=2)
                # cv2.drawMarker(mask_global,point_two,(255,255,255),markerType=2)
            Wrist_FinalCattle,Wrist_FinalBottom=preprocess(Wrist_Cattle=Wrist_Cattle,Wrist_Bottom=Wrist_Bottom)
            pub_cattle.publish(Wrist_FinalCattle)
            pub_bottom.publish(Wrist_FinalBottom)     
            Wrist_Cattle.MyLists.clear()
            Wrist_Bottom.MyLists.clear()
            rate.sleep()
    time4 = time.time()
    # print('************')
    # print('2-1', time2 - time1)
    # print('3-2', time3 - time2)
    # print('4-3', time4 - time3)
    # print('total',time4-time1)
    out_img = im0[:, :, [2, 1, 0]]
    ros_image=out_img
    cv2.imshow('YOLOV5', out_img)
    a = cv2.waitKey(1)
    cv2.imshow("origin",mask_global)
    b=cv2.waitKey(1)
    #### Create CompressedIamge ####
    publish_image(im0)

def image_callback_1(image):
    global ros_image
    global mask_global
    ros_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
    r=ros_image[:,:,0]
    g=ros_image[:,:,1]
    b=ros_image[:,:,2]
    image=cv2.merge([b,g,r])
    #image=cv2.cvtColor(ros_image,cv2.COLOR_BGR2RGB)
    gray=cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
    _,mask=cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)
    mask_global=mask
    # print(ros_image.shape)
    # print(image.shape)
    #print(ros_image.channels())
    with torch.no_grad():
        detect(image)
def publish_image(imgdata):
    image_temp=Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = 'map'
    image_temp.height=IMAGE_HEIGHT
    image_temp.width=IMAGE_WIDTH
    image_temp.encoding='rgb8'
    image_temp.data=np.array(imgdata).tostring()
    #print(imgdata)
    #image_temp.is_bigendian=True
    image_temp.header=header
    image_temp.step=1241*3
    image_pub.publish(image_temp)


if __name__ == '__main__':
    set_logging()
    device = '0'
    device = select_device(device)
    half = device.type != 'cpu'  # half precision only supported on CUDA
    weights = '/home/wangzirui/yolov5/runs/train/exp18/weights/best.pt'
    imgsz = 640
    model = attempt_load(weights, map_location=device)  # load FP32 model
    imgsz = check_img_size(imgsz, s=model.stride.max())  # check img_size
    if half:
        model.half()  # to FP16
    '''
    模型初始化
    '''
    rospy.init_node('ros_yolo')
    image_topic_1 = "/cameras/left_hand_camera/image"
    rospy.Subscriber(image_topic_1, Image, image_callback_1, queue_size=2, buff_size=52428800)
    image_pub = rospy.Publisher('/yolo_result_out', Image, queue_size=1)
    pub_cattle=rospy.Publisher('wrist_cattle',MyListArray,queue_size=100)
    pub_bottom=rospy.Publisher('wrist_bottom',MyListArray,queue_size=100)
    rate=rospy.Rate(20)
    #rospy.init_node("yolo_result_out_node", anonymous=True)
    

    rospy.spin()
