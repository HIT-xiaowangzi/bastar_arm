#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
from unet import UNet
import argparse
import logging
import os

import numpy as np
import torch.nn.functional as F
#from torchvision import transforms

from utils.data_loading import BasicDataset
from unet import UNet
from utils.utils import plot_img_and_mask
model_path='/home/wangzirui/Pytorch-UNet/checkpoints/checkpoint_epoch30.pth'
def predict_img(net,
                full_img,
                device,
                scale_factor=1,
                out_threshold=0.5):
    net.eval()
    img = torch.from_numpy(BasicDataset.preprocess(full_img, scale_factor, is_mask=False))
    img = img.unsqueeze(0)
    img = img.to(device=device, dtype=torch.float32)

    with torch.no_grad():
        output = net(img)

        if net.n_classes > 1:
            probs = F.softmax(output, dim=1)[0]
        else:
            probs = torch.sigmoid(output)[0]

        tf = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((full_img.size[1], full_img.size[0])),
            transforms.ToTensor()
        ])

        full_mask = tf(probs.cpu()).squeeze()

    if net.n_classes == 1:
        return (full_mask > out_threshold).numpy()
    else:
        return F.one_hot(full_mask.argmax(dim=0), net.n_classes).permute(2, 0, 1).numpy()

def mask_to_image(mask):
    if mask.ndim == 2:
        return Image.fromarray((mask * 255).astype(np.uint8))
    elif mask.ndim == 3:
        return Image.fromarray((np.argmax(mask, axis=0) * 255 / mask.shape[0]).astype(np.uint8))


def Callback(imgmsg):

    bridge=CvBridge()
    image=bridge.imgmsg_to_cv2(imgmsg)
    r=image[:,:,0]
    g=image[:,:,1]
    b=image[:,:,2]
    img=cv2.merge([r,g,b])
    image=img.copy()
    net = UNet(n_channels=3, n_classes=3, bilinear=False)

    #device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    device=torch.device('cpu')
    net.to(device=device)
    net.load_state_dict(torch.load(model_path, map_location=device))
    mask = predict_img(net=net,
                           full_img=image,
                           scale_factor=0.5,
                           out_threshold=0.5,
                           device=device)
    result = mask_to_image(mask)
    combine=np.hstack([image,result])
    cv2.imshow("zijixie",image)
    cv2.waitKey(1)
    # k=cv2.waitKey(1)
    # if(k==27):
    #     cv2.imwrite("/home/wangzirui/桌面/3.19腕部相机补充/picture"+str(i)+".jpg",image)
    #     print("picture"+str(i)+".jpg saved successfully!")
    #     i=i+1
def main():
    rospy.init_node("g")
    sub_p=rospy.Subscriber("/cameras/left_hand_camera/image",Image,Callback,queue_size=1,buff_size=100000)
    rospy.Rate(50)
if __name__=="__main__":
    main()
    rospy.spin()