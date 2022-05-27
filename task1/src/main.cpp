#include "ros/ros.h"
#include "std_msgs/String.h"
#include <task1/MyList.h>
#include <task1/MyListArray.h>
#include <task1/GetPoint.h>
#include <vector>
#include <algorithm>
// 接收到订阅的消息后,会进入消息回调函数
const int threshould_1=50;//去除重复框
const int threshould_2=20000;//去除边缘干扰
bool cmp01(task1::MyList msg1,task1::MyList msg2)
{
    return msg1.confidence>msg2.confidence;//比较器，使置信度递减
}
bool cmp02(task1::MyList msg1,task1::MyList msg2)
{
    return msg1.y<msg2.y;//使y坐标递增
}
task1::MyListArray result_block;
task1::MyListArray result_box;
void Callback(const task1::MyListArray msg)
{
// 将接收到的消息打印出来
    ROS_INFO("Enter Callback");
    result_block.MyLists.clear();
    std::vector<task1::MyList> v;
    int length=msg.MyLists.size();
    for(int i=0;i<length;i++)
    {
        v.push_back(msg.MyLists[i]);
    }
    // for(auto it=v.begin();it!=v.end();it++)
    // {
    //     std::cout<<it->x<<" "<<it->y<<" "<<it->class_<<" "<<it->confidence<<" "<<std::endl;
    // }
    //1.针对阴影经常被识别成第六类，做过滤，如果第六类的y坐标不是最高，直接删除
    //std::sort(v.begin(),v.end(),cmp02);
    // if(v.size()>1)
    // {
    //     for(auto it=v.begin()+1;it!=v.end();)
    //     {
    //         if(it->class_==6)
    //         {
    //             it=v.erase(it);
    //             //std::cout<<"进判断"<<std::endl;
    //         }
    //         else
    //         {
    //             it++;
    //         }
    //     }
    // }
    //std::cout<<"开始"<<std::endl;
    //2.按照置信率排序
    std::sort(v.begin(),v.end(),cmp02);
    //3.去除重复框
    int count=0;
    std::vector<task1::MyList>::iterator temp;
    if(!v.empty())
    {
        int i=0;
        for(auto it=v.begin();it!=v.end()&&count<3;count++)
        {
            temp=it+1;
            while(temp!=v.end())
            {
                if(pow(it->x-temp->x,2)+pow(it->y-temp->y,2)<threshould_1||
                    pow(it->x-temp->x,2)+pow(it->y-temp->y,2)>threshould_2)//把距离小于200的框，置信度小的丢掉
                {
                    temp=v.erase(temp);
                }
                else
                {
                    temp++;
                }
            }
            result_block.MyLists.push_back(*it);//把前三置信度的放进最终输出结果里
            it++;
            i++;
        }
    }
    std::sort(result_block.MyLists.begin(),result_block.MyLists.end(),cmp02);//结果再按照y坐标递增
    // for(auto it_2=result.MyLists.begin();it_2!=result.MyLists.end();it_2++)
    // {
        
    //     std::cout<<it_2->x<<" "<<it_2->y<<" "<<it_2->class_<<" "<<it_2->confidence<<" "<<std::endl;
    // }
    //std::cout<<"end of line"<<std::endl;
}

void Callback02(const task1::MyListArray msg)
{
    result_box.MyLists.clear();
    int size=msg.MyLists.size();
    std::vector<task1::MyList> v;
    for(int i=0;i<size;i++)
    {
        v.push_back(msg.MyLists[i]);
    }
    std::sort(v.begin(),v.end(),cmp01);
    if(size>1)
    {
        result_box.MyLists.push_back(v[0]);
        result_box.MyLists.push_back(v[1]);
    }
}
int main(int argc,char** argv)
{
    // 初始化ROS节点
    ros::init(argc,argv,"task1_listener");
    // 创建节点句柄
    ros::NodeHandle n;
    // 创建一个Subscriber,订阅名为chatter的话题,注册回调函数chatterCallback
    ros::Subscriber sub = n.subscribe("xyandclass",1000, Callback);//处理物块信息
    ros::Subscriber sub_box=n.subscribe("box_info",1000,Callback02);//处理箱子信息
    // 循环等待回调函数
    ros::init(argc,argv,"task1_client");
    ros::NodeHandle n1;
    ros::ServiceClient client=n1.serviceClient<task1::GetPoint>("task1_service");
    ros::ServiceClient client_box=n1.serviceClient<task1::GetPoint>("task1_service");
    ros::init(argc,argv,"task1_publisher");
    ros::NodeHandle n2;
    ros::Publisher pub=n2.advertise<geometry_msgs::PoseArray>("first_recognition",100);
    ros::Publisher pub_box=n2.advertise<geometry_msgs::PoseArray>("box_place",100);
    ros::Rate loop_rate(50);
    task1::GetPoint srv;
    while(ros::ok())
    {
        //task1::MyListArray last_result;
        // if(result.MyLists.size()==0)
        // {
        //     result=last_result;//防止在机械臂运动过程中瞬间检测不到物块，造成异常错误
        // }
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
        // last_result=result;
        //result.MyLists.clear();
        srv.request.result.MyLists.clear();
        srv.request.result=result_block;//发送积木块的信息
        if(client.call(srv))
        {
            //ROS_INFO("Yeah!");
            pub.publish(srv.response.Poses);
            loop_rate.sleep();
            ros::spinOnce();
        }
        else
        {
            ROS_INFO("Can't receive the service!");
        }
        ros::spinOnce();
    }
    return 0;
}