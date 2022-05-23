//
// Created by hit-dlr on 2020/10/27.
//

#include "ros/ros.h"
#include <future>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "first_task.h"
#include "third_task.h"
#include "forth_task.h"
#include "camera_info.h"

void send_image(ros::NodeHandle n, const std::string& path)
{
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("/robot/xdisplay", 1);
    cv::Mat image = cv::imread(path, CV_LOAD_IMAGE_COLOR);
    if(image.empty())
        printf("open error\n");
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    for (int i = 0; i < 20; ++i)
    {
        pub.publish(msg);
        ros::Duration(0.01).sleep();
    }
}

int main(int argc, char **argv)
{
    // 可以添加计时器
    // Initialise ROS
    ros::init(argc, argv, "baxter_sim");
    ros::NodeHandle nh;
    ros::NodeHandle nh3;
    ros::NodeHandle nh4;
    ros::AsyncSpinner spinner(8);

    auto *first_task = new FIRST_TASK::FirstTask(nh);
    auto *third_task = new THIRD_TASK::ThirdTask(nh3);
    auto *forth_task = new FORTH_TASK::ForthTask(nh4);

    spinner.start();
    ros::Rate loop_rate(20);

    if (ros::ok())   // 任务一
    {
        std::cout << std::right << "\033[1;31m[Display the image called HIT_100.png. Congrat!]\033[0m" << std::endl;
        send_image(nh, "/home/hit/wrc_hit/src/wrc_hit_lib/image/HIT_100.png");//baxterworking   HIT_100

        std::cout << std::right << "\033[1;31m[Connecting to ROS Master]\033[0m" << std::endl;
        ros::Duration(2).sleep();
        std::cout << std::right << "\033[1;32m[Connecting Finished]\033[0m" << std::endl;
        ros::Duration(2).sleep();

        if(!first_task->setup())
        {
            std::cout << std::right << "\033[1;31m[Failed to setup the first task]\033[0m" << std::endl;
            return 0;
        }

        first_task->start();

        ros::Duration(3).sleep();
        std::cout << std::left << "\033[1;32m[All Simulations Finish]\033[0m" << std::endl;
    }
    spinner.stop();
    delete first_task;
    //////////////////////////////////任务三/////////////////////////////////////
//    if (ros::ok())   // 任务三
//    {
//        std::cout << std::right << "\033[1;31m[Display the image called HIT_100.png. Congrat!]\033[0m" << std::endl;
//        send_image(nh, "/home/hit/wrc_hit/src/wrc_hit_lib/image/HIT_100.png");//baxterworking   HIT_100
//
//        std::cout << std::right << "\033[1;31m[Connecting to ROS Master]\033[0m" << std::endl;
//        ros::Duration(2).sleep();
//        std::cout << std::right << "\033[1;32m[Connecting Finished]\033[0m" << std::endl;
//        ros::Duration(2).sleep();
//
//        if(!third_task->setup())
//        {
//            std::cout << std::right << "\033[1;31m[Failed to setup the third task]\033[0m" << std::endl;
//            return 0;
//        }
//
//        third_task->start();
//
//        ros::Duration(3).sleep();
//        std::cout << std::left << "\033[1;32m[All Simulations Finish]\033[0m" << std::endl;
//    }
//    spinner.stop();
//    delete third_task;
    // 可以添加计时器
    //////////////////////////////////任务四/////////////////////////////////////
//    if (ros::ok())   // 任务四
//    {
//        std::cout << std::right << "\033[1;31m[Display the image called HIT_100.png. Congrat!]\033[0m" << std::endl;
//        send_image(nh, "/home/hit/wrc_hit/src/wrc_hit_lib/image/HIT_100.png");   //baxterworking   HIT_100
//
//        std::cout << std::right << "\033[1;31m[Connecting to ROS Master]\033[0m" << std::endl;
//        ros::Duration(2).sleep();
//        std::cout << std::right << "\033[1;32m[Connecting Finished]\033[0m" << std::endl;
//        ros::Duration(2).sleep();
//
//        if(!forth_task->setup())
//        {
//            std::cout << std::right << "\033[1;31m[Failed to setup the forth task]\033[0m" << std::endl;
//            return 0;
//        }
//
//        forth_task->start();
//
//        ros::Duration(3).sleep();
//        std::cout << std::left << "\033[1;32m[All Simulations Finish]\033[0m" << std::endl;
//    }
//
//    spinner.stop();
//    delete forth_task;
//     可以添加计时器
    return 1;
}
