//
// Created by hit on 2022/2/24.
//

#ifndef WRC_HIT_LIB_REALSENSE_INFO_H
#define WRC_HIT_LIB_REALSENSE_INFO_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>   // 很重要
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include "MyList.h"
#include "MyListArray.h"
class RealsenseInfo
{
public:
    RealsenseInfo(ros::NodeHandle nh);
    ~RealsenseInfo();
    void blockInfoCallback(const geometry_msgs::PoseArrayConstPtr & camera_msg);
    void boxInfoCallback(const geometry_msgs::PoseArrayConstPtr & box_msg);
    void headInfoCallback(const geometry_msgs::PoseArrayConstPtr & head_msg);
    void leftcameraInfoCallback(const std_msgs::Float32MultiArrayPtr &camera_msg);
    void rightcameraInfoCallback(const std_msgs::Float32MultiArrayPtr &camera_msg);
    void boxWidthCallback(const std_msgs::Int16ConstPtr width_msg);
    void wristcattleCallback(const task1::MyListArrayConstPtr &wrist_cattle_msg);
    void wristbottomCallback(const task1::MyListArrayConstPtr &wrist_bottom_msg);
    std::vector<double> getRightCompensationInfo();
    std::vector<double> getLeftCompensationInfo();
    std::vector<std::vector<double>> getBlockInfo();
    std::vector<double> getBoxInfo();
    std::vector<std::vector<double>> getHeadInfo();
    int getWidthInfo();


private:
    ros::NodeHandle nh;
    ros::Subscriber block_info_sub,box_info_sub,lefthand_camera_info_sub,righthand_camera_info_sub,
                    box_width_sub,head_info_sub,
                    wrist_cattle_sub,wrist_bottom_sub;
    bool blockisReceive,boxisReceive,headisReceive;
    double blockpos[3][6]; //第一问的积木位置
    double headpos[3][6];
    double boxpos[6];
    double wrist_cattle[2];//离中心最近的牛的中心坐标
    double wrist_bottom[2];//离wrist_cattle最近的屁股坐标
    int box_width;
    int method;//method=0:使用模板匹配获取 method=1:使用腕部相机+yolo获取
    std::vector<double> left_compensation, right_compensation;

};

#endif //WRC_HIT_LIB_REALSENSE_INFO_H