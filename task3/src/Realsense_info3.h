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
#include "task3/MyList.h"
#include "task3/MyListArray.h"
class RealsenseInfo3
{
public:
    RealsenseInfo3(ros::NodeHandle nh);
    ~RealsenseInfo3();
    void BlueRulerCallback(const geometry_msgs::PoseArrayConstPtr & blue_msg);
    void boxInfoCallback(const geometry_msgs::PoseArrayConstPtr & box_msg);
    void RedRulerCallback(const geometry_msgs::PoseArrayConstPtr & red_msg);
    void leftcameraInfoCallback(const std_msgs::Float32MultiArrayPtr &camera_msg);
    void rightcameraInfoCallback(const std_msgs::Float32MultiArrayPtr &camera_msg);
    void wristbluerulerCallback(const task3::MyListArrayConstPtr &wrist_ruler_msg);
    void wristredrulerCallback(const task3::MyListArrayConstPtr &wrist_ruler_msg);
    void wristboxCallback(const task3::MyListArrayConstPtr &wrist_box_msg);
    void AngleCallback(const std_msgs::Float32ConstPtr &msg);
    std::vector<double> getBlueCompensationInfo();
    std::vector<double> getRedCompensationInfo();
    std::vector<double> getRightCompensationInfo();
    std::vector<std::vector<double>> getBlueRulerInfo();
    std::vector<double> getBoxInfo();
    std::vector<std::vector<double>> getRedRulerInfo();
    double getAngle();


private:
    ros::NodeHandle nh;
    ros::Subscriber blueruler_sub,box_info_sub,lefthand_camera_info_sub,righthand_camera_info_sub,
                    redruler_sub,wrist_blueruler_sub,wrist_redruler_sub,wrist_box_sub,angle_sub;
    bool blueisReceive,boxisReceive,redisReceive;
    double bluerulerpos[2][6];
    double redrulerpos[2][6];
    double boxpos[6];
    double wrist_blueruler[2];//蓝色魔尺的中心点
    double wrist_redruler[2];//红色魔尺的中心点
    double wrist_box[2];
    double ratio_blue;//蓝色魔尺的宽高比
    double ratio_red;//红色魔尺的宽高比
    double angle;
    std::vector<double> left_compensation, right_compensation;

};

// #endif //WRC_HIT_LIB_REALSENSE_INFO_H