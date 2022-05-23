//
// Created by hit on 2020/11/17.
//

#include <thread>
#include <future>
#include "camera_info.h"
#include <std_msgs/Int16.h>   // 很重要
#include <std_msgs/Float32MultiArray.h>   // 很重要

#define L_ARM "left"
#define R_ARM "right"

CameraInfo::CameraInfo(ros::NodeHandle nh)
{
    this->nh = nh;
    std::cout << std::setw(80) << std::left << "Registering Realsense information callback: ";
    camera_info_sub = nh.subscribe("/lefttop_point", 1000, &CameraInfo::cameraInfoCallback, this);  // 订阅物体信息
    if(camera_info_sub == nullptr)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::left << "\033[1;32m[OK]\033[0m" << std::endl;

    std::cout << std::setw(80) << std::left << "Registering Kinect V2  publisher: ";
    camera_pub = nh.advertise<std_msgs::Int16>("/camera_pub", 5);   // 整型
    if(camera_pub == nullptr)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;

//    std::cout << std::setw(80) << std::left << "Registering camera_info_move_sub  publisher: ";
//    camera_info_move_sub = nh.subscribe("/lefttop_point", 1000, &CameraInfo::MoveCallback, this);  // 抓取移动物体,接收信息后开始移动,话题名称未定
//    if(camera_info_move_sub == nullptr)
//    {
//        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
//        return;
//    }
//    std::cout << std::left << "\033[1;32m[OK]\033[0m" << std::endl;

    std::cout << std::setw(80) << std::left << "Registering right and left hand_camera: ";
    lefthand_camera_info_sub = nh.subscribe("/left_hand_camera", 1, &CameraInfo::leftcameraInfoCallback, this);  // 订阅左手相机话题信息
    righthand_camera_info_sub = nh.subscribe("/right_hand_camera", 1, &CameraInfo::rightcameraInfoCallback, this);  // 订阅右手相机话题信息
    if(lefthand_camera_info_sub == nullptr || righthand_camera_info_sub == nullptr)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::left << "\033[1;32m[OK]\033[0m" << std::endl;

    std::cout << std::setw(80) << std::left << "Registering forth_camera_info_move_sub  publisher: ";
    forth_camera_info_move_sub = nh.subscribe("/gesture", 1000, &CameraInfo::forthInfoCallback, this);  //
    if(forth_camera_info_move_sub == nullptr)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::left << "\033[1;32m[OK]\033[0m" << std::endl;

//    wide_cup_info_sub = nh.subscribe("/cup1", 5, &CameraInfo::WidecupInfoCallback, this);  // 任务三:订阅宽口杯子的位置信息
//    if(wide_cup_info_sub == nullptr)
//    {
//        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
//        return;
//    }
//    std::cout << std::left << "\033[1;32m[OK]\033[0m" << std::endl;

    narrow_cup_info_sub = nh.subscribe("/cup", 5, &CameraInfo::NarrowcupInfoCallback, this);  // 任务三:订阅窄口杯子的位置信息和补偿信息
    if(narrow_cup_info_sub == nullptr)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::left << "\033[1;32m[OK]\033[0m" << std::endl;

    test_cup_info_sub = nh.subscribe("/flag", 5, &CameraInfo::taskthirdInfoCallback, this);  // 任务三:立方体移动测试
    if(test_cup_info_sub == nullptr)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::left << "\033[1;32m[OK]\033[0m" << std::endl;

    left_compensation.resize(3);
    right_compensation.resize(3);
    narrow_cup_position.resize(6);
}

CameraInfo::~CameraInfo()
{
}

void CameraInfo::cameraInfoCallback(const geometry_msgs::PoseArrayConstPtr & camera_msg)  // (任务一,获取五个普通物体的位置和角度信息)
{
    for (int i = 0; i < camera_msg->poses.size(); ++i)
    {
        objpos[i][0] = camera_msg->poses[i].position.x;
        objpos[i][1] = camera_msg->poses[i].position.y;
        objpos[i][2] = camera_msg->poses[i].position.z;
        objpos[i][3] = camera_msg->poses[i].orientation.x;
        objpos[i][4] = camera_msg->poses[i].orientation.y;
    }
}

void CameraInfo::leftcameraInfoCallback(const std_msgs::Float32MultiArrayPtr &left_camera_msg)  // (任务一,左手摄像头获取角度和位置补偿信息)
{
    left_compensation[0] = left_camera_msg->data.at(2);        // 角度补偿
    left_compensation[1] = -left_camera_msg->data.at(0);       // x补偿
    left_compensation[2] = -left_camera_msg->data.at(1);;      // y补偿
//    std::cout<< "____________________" << camera_msg->data.at(0) << ", "<< camera_msg->data.at(1) << ", "<< camera_msg->data.at(2)<< std::endl;
}

void CameraInfo::rightcameraInfoCallback(const std_msgs::Float32MultiArrayPtr &right_camera_msg)  // (任务一,右手摄像头获取角度和位置补偿信息)
{
    right_compensation[0] = right_camera_msg->data.at(2);     // 角度补偿
    right_compensation[1] = -right_camera_msg->data.at(0);    // x补偿
    right_compensation[2] = -right_camera_msg->data.at(1);    // y补偿
}

std::vector<double> CameraInfo::getRightCompensationInfo()  // 补偿信息,包含角度和距离(任务一)
{
    std::vector<double> compen_r;
    compen_r.resize(3);
    compen_r.clear();
    for (int i = 0; i < 3; ++i)
        compen_r[i] = this->right_compensation[i]; // 角度, x方向,y方向
    return compen_r;
}

std::vector<double> CameraInfo::getLeftCompensationInfo()  // 补偿信息,包含角度和距离(任务一)
{
    std::vector<double> compen_l;
    compen_l.resize(3);
    compen_l.clear();
    for (int i = 0; i < 3; ++i)
        compen_l[i] = this->left_compensation[i]; // 角度, x方向, y方向
    return compen_l;
}


std::vector<std::vector<double>> CameraInfo::getCameraInfo()  // 得到普通物体的信息,不包含重叠物体(任务一)
{
    std::vector<std::vector<double>> objPos;
    objPos.resize(5);
    objPos.clear();
    std::vector<double> objPosTemp;
    objPosTemp.clear();
    objPosTemp.resize(5);
    for (int i = 2; i < 7; ++i)
    {
        objPosTemp.clear();
        for (int j = 0; j < 5; ++j)
            objPosTemp.push_back(objpos[i][j]);
        objPos.push_back(objPosTemp);
    }
    return objPos;
}

std::vector<std::vector<double>> CameraInfo::getCenterCircleInfo()  // 得到圆盘中心的位置(任务一)
{
    std::vector<std::vector<double>> centercircle;
    centercircle.clear();
    centercircle.push_back({this->objpos[7][0],this->objpos[7][1],this->objpos[7][2],this->objpos[7][4]});  // 圆盘中心位置xyz和圆盘上物体的标号
    return centercircle;                                                                                    // 标号代表物体是平躺还是竖直,决定向下抓取等待时间的不同
}

std::vector<std::vector<double>> CameraInfo::getCameraOverlapInfo()  // 得到重叠物体的信息, 在消息中位于前两位(任务一)
{
    std::vector<std::vector<double>> OverlapobjPos;
    OverlapobjPos.resize(2);
    OverlapobjPos.clear();
    std::vector<double> objPosTemp;
    objPosTemp.clear();
    objPosTemp.resize(5);
    for (int i = 0; i < 2; ++i)
    {
        objPosTemp.clear();
        for (int j = 0; j < 5; ++j)
            objPosTemp.push_back(objpos[i][j]);
        OverlapobjPos.push_back(objPosTemp);
    }
    return OverlapobjPos;
}

//void CameraInfo::cameraCmdPub(const int order) // (任务一,发布消息,已经移动到圆盘边缘的位置,暂未采用)
//{
//    std_msgs::Int16 camera_order;
//    camera_order.data = order;
//    camera_pub.publish(camera_order);
//}

//void CameraInfo::MoveCallback(const geometry_msgs::PoseArrayConstPtr & camera_msg)  //(任务一,抓取移动的物体,暂未采用)
//{
//    order = camera_msg->poses[1].position.x;   // 未定
//}

//void CameraInfo::WidecupInfoCallback(const geometry_msgs::PoseConstPtr &cup1_camera_msg)  // (任务三,获取宽口杯子的位置信息,话题名称未定)
//{
//    wide_cup_position[0] = cup1_camera_msg->position.x;        // 宽口杯子位置信息 x
//    wide_cup_position[1] = cup1_camera_msg->position.y;        // 宽口杯子位置信息 y
//    wide_cup_position[2] = cup1_camera_msg->position.z;        // 宽口杯子位置信息 z
//}

void CameraInfo::NarrowcupInfoCallback(const geometry_msgs::PoseConstPtr &cup_camera_msg)  // (任务三,获取窄口杯子的位置信息,话题名称未定)
{
    narrow_cup_position[0] = cup_camera_msg->position.x;        // 窄口杯子位置信息 x
    narrow_cup_position[1] = cup_camera_msg->position.y;        // 窄口杯子位置信息 y
    narrow_cup_position[2] = cup_camera_msg->position.z;        // 窄口杯子位置信息 z
    narrow_cup_position[3] = cup_camera_msg->orientation.x;     // 窄口杯子位置补偿信息 x
    narrow_cup_position[4] = cup_camera_msg->orientation.y;     // 窄口杯子位置补偿信息 y
    narrow_cup_position[5] = cup_camera_msg->orientation.z;     // 窄口杯子位置补偿信息 z
}

std::vector<double> CameraInfo::getWideCupInfo()      // 返回宽口杯子的位置(任务三)
{
    std::vector<double> WideCupPos;
    WideCupPos.resize(3);
    WideCupPos.clear();
    for (int i = 0; i < 3; ++i)
        WideCupPos[i] = this->wide_cup_position[i];
    return WideCupPos;
}

std::vector<double> CameraInfo::getNarrowCupInfo()      // 返回窄口杯子的位置信息和补偿信息(任务三)
{
    std::vector<double> NarrowCupPos;
    NarrowCupPos.resize(7);
    NarrowCupPos.clear();
    for (int i = 0; i < 6; ++i)
        NarrowCupPos[i] = this->narrow_cup_position[i];
    return NarrowCupPos;
}

void CameraInfo::forthInfoCallback(const std_msgs::Float32MultiArrayPtr &forth_camera_msg)  // (task four)
{
    forth_task[0] = forth_camera_msg->data.at(0);     //
    forth_task[1] = forth_camera_msg->data.at(1);     //
    forth_task[2] = forth_camera_msg->data.at(2);;    //
}

std::vector<double> CameraInfo::getForthCompensationInfo()  // (task four)
{
    std::vector<double> forth_info;
    forth_info.resize(3);
    forth_info.clear();
    for (int i = 0; i < 3; ++i)
        forth_info[i] = this->forth_task[i]; //
    return forth_info;
}

void CameraInfo::taskthirdInfoCallback(const std_msgs::Float32ConstPtr& third_test_camera_msg)  // (任务三立方体测试)
{
    test_temp = third_test_camera_msg->data;//
//    std::cout << " test_temp ： " << test_temp << std::endl;
}

double CameraInfo::gettestorderInfo()      // (任务三遍历点测试)
{
    double order_info;
    order_info = this->test_temp;
    this->test_temp = 0;
    return order_info;
}
