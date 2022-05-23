//
// Created by hit on 2020/11/17.
//

#ifndef WRC_HIT_LIB_CAMERA_INFO_H
#define WRC_HIT_LIB_CAMERA_INFO_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>   // 很重要
#include <std_msgs/Float32.h>

class CameraInfo
{
public:
    CameraInfo(ros::NodeHandle nh);
    ~CameraInfo();
    void cameraInfoCallback(const geometry_msgs::PoseArrayConstPtr & jnt_msg);
    void leftcameraInfoCallback(const std_msgs::Float32MultiArrayPtr &camera_msg);
    void rightcameraInfoCallback(const std_msgs::Float32MultiArrayPtr &camera_msg);
    void forthInfoCallback(const std_msgs::Float32MultiArrayPtr &forth_camera_msg);  // (task four)
    void WidecupInfoCallback(const geometry_msgs::PoseConstPtr &cup_camera_msg);
    void NarrowcupInfoCallback(const geometry_msgs::PoseConstPtr &cup_camera_msg);
    void taskthirdInfoCallback(const std_msgs::Float32ConstPtr& third_test_camera_msg);  // (任务三立方体测试)
    std::vector<double> getRightCompensationInfo();
    std::vector<double> getLeftCompensationInfo();
    std::vector<double> getForthCompensationInfo();  // (task four)
    std::vector<std::vector<double>> getCameraOverlapInfo();
    std::vector<std::vector<double>> getCameraInfo();
    std::vector<std::vector<double>> getCenterCircleInfo();
    std::vector<double> getWideCupInfo();  // (任务三)
    std::vector<double> getNarrowCupInfo();  // (任务三)
    double gettestorderInfo();   // (任务三立方体测试)
//    void cameraCmdPub(const int order);
//    void MoveCallback(const geometry_msgs::PoseArrayConstPtr & camera_msg);



private:
    ros::NodeHandle nh;
    ros::Subscriber camera_info_sub, camera_info_move_sub, lefthand_camera_info_sub,
            righthand_camera_info_sub, forth_camera_info_move_sub, wide_cup_info_sub,
            narrow_cup_info_sub, test_cup_info_sub ;//, cup_info_sub;
    ros::Publisher camera_pub;
    double objpos[8][5];  // 8个物体(任务一)
    double test_temp;
    std::vector<double> left_compensation, right_compensation;
    std::vector<double> forth_task, wide_cup_position, narrow_cup_position;
    int order;
};

#endif //WRC_HIT_LIB_CAMERA_INFO_H
