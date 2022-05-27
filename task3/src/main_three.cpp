#include "ros/ros.h"
#include "std_msgs/String.h"
#include <task3/MyList.h>
#include <task3/MyListArray.h>
#include <task3/GetPoint.h>
#include <vector>
#include <algorithm>
// 接收到订阅的消息后,会进入消息回调函数
const int threshould_1=50;//去除重复框
const int threshould_2=20000;//去除边缘干扰
bool cmp01(task3::MyList msg1,task3::MyList msg2)
{
    return msg1.confidence>msg2.confidence;//比较器，使置信度递减
}
bool cmp02(task3::MyList msg1,task3::MyList msg2)
{
    return msg1.y<msg2.y;//使y坐标递增
}
task3::MyListArray result_box;
task3::MyListArray result_blueruler;
task3::MyListArray result_redruler;
void Callback(const task3::MyListArray msg)
{
    ROS_INFO("Hi!I'm wangzirui!");
    result_box.MyLists.clear();
    int size=msg.MyLists.size();
    std::vector<task3::MyList> v;
    for(int i=0;i<size;i++)
    {
        v.push_back(msg.MyLists[i]);
    }
    if(size>0)
    {
        result_box.MyLists.push_back(v[0]);
    }
}

void Callback02(const task3::MyListArray msg)
{
    result_blueruler.MyLists.clear();
    int size=msg.MyLists.size();
    std::vector<task3::MyList> v;
    for(int i=0;i<size;i++)
    {
        v.push_back(msg.MyLists[i]);
    }
    std::sort(v.begin(),v.end(),cmp01);
    int temp=std::min(2,size);
    for(int i=0;i<temp;i++)
    {
        result_blueruler.MyLists.push_back(v[i]);//限制个数，防止外加干扰使得蓝尺的数目多于2，造成数组越界
    }
}
void Callback03(const task3::MyListArray msg)
{
    result_redruler.MyLists.clear();
    int size=msg.MyLists.size();
    std::vector<task3::MyList> v;
    for(int i=0;i<size;i++)
    {
        v.push_back(msg.MyLists[i]);
    }
    std::sort(v.begin(),v.end(),cmp01);
    int temp=std::min(2,size);
    for(int i=0;i<temp;i++)
    {
        result_redruler.MyLists.push_back(msg.MyLists[i]);//同样限制个数，防止数组越界
    }
}
int main(int argc,char** argv)
{
    // 初始化ROS节点
    ros::init(argc,argv,"task3_listener");
    // 创建节点句柄
    ros::NodeHandle n;
    // 创建一个Subscriber,订阅名为chatter的话题,注册回调函数chatterCallback
    ros::Subscriber sub_box= n.subscribe("box",1000, Callback);//处理物块信息
    ros::Subscriber sub_blueruler=n.subscribe("blueruler",1000,Callback02);//处理箱子信息
    ros::Subscriber sub_redruler=n.subscribe("redruler",1000,Callback03);//处理箱子信息
    // 循环等待回调函数
    ros::init(argc,argv,"task3_client");
    ros::NodeHandle n1;
    ros::ServiceClient client_box=n1.serviceClient<task3::GetPoint>("task3_service");
    ros::ServiceClient client_blueruler=n1.serviceClient<task3::GetPoint>("task3_service");
    ros::ServiceClient client_redruler=n1.serviceClient<task3::GetPoint>("task3_service");
    ros::init(argc,argv,"task3_publisher");
    ros::NodeHandle n2;
    ros::Publisher pub_box=n2.advertise<geometry_msgs::PoseArray>("box_place",100);
    ros::Publisher pub_blueruler=n2.advertise<geometry_msgs::PoseArray>("blueruler_place",100);
    ros::Publisher pub_redruler=n2.advertise<geometry_msgs::PoseArray>("redruler_place",100);
    ros::Rate loop_rate(50);
    task3::GetPoint srv;
    while(ros::ok())
    {
        srv.request.result=result_box;//发送箱子的信息
        if(client_box.call(srv))
        {
           //ROS_INFO("Yeah!");
            pub_box.publish(srv.response.Poses);
            loop_rate.sleep();
            ros::spinOnce();
        }
        else
        {
            ROS_INFO("Can't receive box info!");
        }
        srv.request.result.MyLists.clear();
        srv.request.result=result_blueruler;//发送积木块的信息
        if(client_blueruler.call(srv))
        {
            //ROS_INFO("Yeah!");
            pub_blueruler.publish(srv.response.Poses);
            loop_rate.sleep();
            ros::spinOnce();
        }
        else
        {
            ROS_INFO("Can't receive the blueruler!");
        }
        srv.request.result.MyLists.clear();
        srv.request.result=result_redruler;
        if(client_redruler.call(srv))
        {
            //ROS_INFO("Yeah!");
            pub_redruler.publish(srv.response.Poses);
            loop_rate.sleep();
            ros::spinOnce();
        }
        else
        {
            ROS_INFO("Can't receive the redruler!");
        }
        ros::spinOnce();
    }
    return 0;
}