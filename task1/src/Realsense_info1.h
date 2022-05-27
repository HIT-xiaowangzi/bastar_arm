//
// Created by hit on 2022/2/24.
//

// #ifndef WRC_HIT_LIB_REALSENSE_INFO_H
// #define WRC_HIT_LIB_REALSENSE_INFO_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>   // 很重要
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include "task1/MyList.h"
#include "task1/MyListArray.h"
class RealsenseInfo1
{
public:
    RealsenseInfo1(ros::NodeHandle nh);
    ~RealsenseInfo1();
    void blockInfoCallback(const geometry_msgs::PoseArrayConstPtr & camera_msg);
    void boxInfoCallback(const geometry_msgs::PoseArrayConstPtr & box_msg);
    void leftcameraInfoCallback(const std_msgs::Float32MultiArrayPtr &camera_msg);
    void rightcameraInfoCallback(const std_msgs::Float32MultiArrayPtr &camera_msg);
    void wristboxCallback(const task1::MyListArrayConstPtr &wrist_box_msg);
    
    std::vector<double> getRightCompensationInfo();
    std::vector<double> getLeftCompensationInfo();
    std::vector<std::vector<double>> getBlockInfo();
    std::vector<double> getBoxInfo();
    

private:
    ros::NodeHandle nh;
    ros::Subscriber block_info_sub,box_info_sub,lefthand_camera_info_sub,righthand_camera_info_sub,wrist_box_sub;
                    
    bool blockisReceive,boxisReceive,headisReceive;
    double blockpos[3][6]; //第一问的积木位置
    //double headpos[3][6];
    double boxpos[6];
    double wrist_box[2];
    int method;//method=0:使用模板匹配获取 method=1:使用腕部相机+yolo获取
    std::vector<double> left_compensation, right_compensation;

};

// #endif //WRC_HIT_LIB_REALSENSE_INFO_H