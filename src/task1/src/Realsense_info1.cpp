//
// Created by hit on 2022/2/24.
//

#include <thread>
#include <future>
#include "Realsense_info1.h"
#include <std_msgs/Int16.h>   // 很重要
#include <std_msgs/Float32MultiArray.h>   // 很重要

#define L_ARM "left"
#define R_ARM "right"
const double PI=3.1415926535897;
// int MinInd(int* arr,int len)//返回数组的最小下标
// {
//     int min_ind=0;
//     int min_val=0;
//     if(len>0)
//     {
//         min_val=arr[0];
//     }
//     for(int i=1;i<len;i++)
//     {
//         if(arr[i]<min_val)
//         {
//             min_ind=i;
//             min_val=arr[i];
//         }
//     }
//     return min_ind;
// }
RealsenseInfo1::RealsenseInfo1(ros::NodeHandle n)
{
    this->nh=n;
    this->blockisReceive=false;
    this->boxisReceive=false;
    this->headisReceive=false;
    //this->box_width=400;
    this->method=0;//0:模板匹配获取中心 1：腕部yolo获取中心
    ROS_INFO("camera_3");
    block_info_sub=this->nh.subscribe("first_recognition",100,&RealsenseInfo1::blockInfoCallback,this);
    ROS_INFO("camera_4");
    box_info_sub=this->nh.subscribe("box_place",100,&RealsenseInfo1::boxInfoCallback,this);
    ROS_INFO("camera_5");
    //head_info_sub=this->nh.subscribe("head_place",100,&RealsenseInfo1::headInfoCallback,this);
    ROS_INFO("camera_6");
    lefthand_camera_info_sub=this->nh.subscribe("left_hand_camera",2,&RealsenseInfo1::leftcameraInfoCallback,this);
    ROS_INFO("camera_7");
    righthand_camera_info_sub=this->nh.subscribe("right_hand_camera",2,&RealsenseInfo1::rightcameraInfoCallback,this);
    ROS_INFO("camera_8");
    //box_width_sub=this->nh.subscribe("width",2,&RealsenseInfo1::boxWidthCallback,this);
    ROS_INFO("camera_9");
    ROS_INFO("camera_1");
    //wrist_cattle_sub=this->nh.subscribe("wrist_cattle",2,&RealsenseInfo1::rightcameraInfoCallback,this);
    //wrist_bottom_sub=this->nh.subscribe("wrist_bottom",2,&RealsenseInfo1::rightcameraInfoCallback,this);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<6;j++)
        {
            blockpos[i][j]=0;
            //headpos[i][j]=0;
        }
    }
    for(int k=0;k<6;k++)
    {
        boxpos[k]=0;
    }
    left_compensation.resize(3);
    left_compensation.clear();
    right_compensation.resize(3);
    right_compensation.clear();
    ROS_INFO("camera_2");
    //wrist_cattle[0]=0;
    //wrist_cattle[1]=0;
    //wrist_bottom[0]=0;
    //wrist_bottom[1]=0;
    ros::spinOnce();
}

RealsenseInfo1::~RealsenseInfo1()
{

}

void RealsenseInfo1::blockInfoCallback(const geometry_msgs::PoseArrayConstPtr & camera_msg)
{
    if(camera_msg->poses.size()==0)
    {
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<6;j++)
            {
                blockpos[i][j]=0;//没有的时候清零，防止陷入死循环
            }
        }
        return;
    }
    if(camera_msg->poses[0].position.x>0 &&camera_msg->poses[0].position.x<2)
    {
        this->blockisReceive=true;
        for (int i = 0; i < camera_msg->poses.size(); i++)
        {
            blockpos[i][0] = camera_msg->poses[i].position.x;
            blockpos[i][1] = camera_msg->poses[i].position.y;
            //blockpos[i][2] = camera_msg->poses[i].position.z+0.15;
            blockpos[i][2] = 0.07;
            blockpos[i][3] = 3.14;
            blockpos[i][4] = 0;
            blockpos[i][5] = 3.14;
        }
        for(int j=camera_msg->poses.size();j<3;j++)
        {
            blockpos[j][0]=0;
            blockpos[j][1]=0;
            blockpos[j][2]=0;
            blockpos[j][3]=0;
            blockpos[j][4]=0;
            blockpos[j][5]=0;
        }
    }
    else
    {
        ROS_INFO("Camera info isn't valid!");
    }
}

void RealsenseInfo1::boxInfoCallback(const geometry_msgs::PoseArrayConstPtr & box_msg)
{
    if(box_msg->poses.size()==0)
    {
        // for(int i=0;i<6;i++)
        // {
        //     boxpos[i]=0;
        // }
        return;
    }
    if(box_msg->poses[0].position.x>0 &&box_msg->poses[0].position.x<2)
    {
        this->boxisReceive=true;
        // if(abs(box_msg->poses[0].position.x-box_msg->poses[1].position.x)>0.05)//拍到两个不同面
        // {
        //     boxpos[0]=(box_msg->poses[0].position.x+box_msg->poses[1].position.x)/2;
        //     boxpos[1]=(box_msg->poses[0].position.y+box_msg->poses[1].position.y)/2;
        //     boxpos[2]=0.30;//举高一点，便于补偿
        //     boxpos[3]=3.14;
        //     boxpos[4]=0;
        //     boxpos[5]=3.14;
        // }
        // else
        // {
        boxpos[0]=box_msg->poses[0].position.x+0.105;//全部拍到前面
        boxpos[1]=box_msg->poses[0].position.y;
        boxpos[2]=0.30;//举高一点，便于补偿
        boxpos[3]=3.14;
        boxpos[4]=0;
        boxpos[5]=3.14;
        //}
            
    }
    else
    {
        ROS_INFO("Camera info isn't valid!");
    }
}

// void RealsenseInfo1::headInfoCallback(const geometry_msgs::PoseArrayConstPtr & head_msg)
// {
//     if(head_msg->poses.size()==0)
//     {
//         for(int i=0;i<3;i++)
//         {
//             for(int j=0;j<6;j++)
//             {
//                 headpos[i][j]=0;//没有的时候清零，防止陷入死循环
//             }
//         }
//         return;
//     }
//     if(head_msg->poses[0].position.x>0 &&head_msg->poses[0].position.x<2)
//     {
//         this->blockisReceive=true;
//         for (int i = 0; i < head_msg->poses.size(); i++)
//         {
//             blockpos[i][0] = head_msg->poses[i].position.x;
//             blockpos[i][1] = head_msg->poses[i].position.y;
//             //blockpos[i][2] = camera_msg->poses[i].position.z+0.15;
//             blockpos[i][2] = 0.02;
//             blockpos[i][3] = 3.14;
//             blockpos[i][4] = 0;
//             blockpos[i][5] = 3.14;
//         }
//         for(int j=head_msg->poses.size();j<3;j++)
//         {
//             blockpos[j][0]=0;
//             blockpos[j][1]=0;
//             blockpos[j][2]=0;
//             blockpos[j][3]=0;
//             blockpos[j][4]=0;
//             blockpos[j][5]=0;
//         }
//     }
//     else
//     {
//         ROS_INFO("Head info isn't valid!");
//     }
// }

void RealsenseInfo1::leftcameraInfoCallback(const std_msgs::Float32MultiArrayPtr &left_camera_msg)
{
    left_compensation[0] = left_camera_msg->data.at(2);        // 角度补偿
    left_compensation[1] = left_camera_msg->data.at(0);       // x补偿
    left_compensation[2] = left_camera_msg->data.at(1);;      // y补偿
}

void RealsenseInfo1::rightcameraInfoCallback(const std_msgs::Float32MultiArrayPtr &right_camera_msg)
{
    right_compensation[0] = right_camera_msg->data.at(2);     // 角度补偿
    right_compensation[1] = right_camera_msg->data.at(0);    // x补偿
    right_compensation[2] = right_camera_msg->data.at(1);    // y补偿
}

// void RealsenseInfo1::boxWidthCallback(const std_msgs::Int16ConstPtr width_msg)
// {
//     if(width_msg==NULL||width_msg->data==0)
//     {
//         return;
//     }
//     else
//     {
//         box_width=width_msg->data;
//     }
// }

// void RealsenseInfo1::wristcattleCallback(const task1::MyListArrayConstPtr &wrist_cattle_msg)
// {
//     int size=wrist_cattle_msg->MyLists.size();
//     if(size==0)
//     {
//         return;
//     }
//     else
//     {
//         int* dist=new int[size];
//         for(int i=0;i<size;i++)
//         {
//             dist[i]=pow((wrist_cattle_msg->MyLists[i].x-340),2)+pow((wrist_cattle_msg->MyLists[i].y-190),2);
//         }
//         int min_ind=MinInd(dist,size);
//         wrist_cattle[0]=wrist_cattle_msg->MyLists[min_ind].x;
//         wrist_cattle[1]=wrist_cattle_msg->MyLists[min_ind].y;
//         delete dist;
        
//     }
    
// }
// void RealsenseInfo1::wristbottomCallback(const task1::MyListArrayConstPtr &wrist_bottom_msg)
// {
//     int size=wrist_bottom_msg->MyLists.size();
//     if(size==0)
//     {
//         return;
//     }
//     else
//     {
//         int *dist2=new int[size];
//         int x_center,y_center;
//         if(wrist_cattle[0]!=0&&wrist_cattle[1]!=0)//牛的信息已经传到了
//         {
//             x_center=wrist_cattle[0];
//             y_center=wrist_cattle[1];
//         }
//         else//牛的信息尚未传到
//         {
//             x_center=340;
//             y_center=190;
//         }
//         for(int i=0;i<size;i++)
//         {
//             dist2[i]=pow((wrist_bottom_msg->MyLists[i].x-x_center),2)+pow((wrist_bottom_msg->MyLists[i].y-y_center),2);
//         }
//         int min_ind=MinInd(dist2,size);
//         wrist_cattle[0]=wrist_bottom_msg->MyLists[min_ind].x;
//         wrist_cattle[1]=wrist_bottom_msg->MyLists[min_ind].y;
//         delete dist2;
//     }
// }
std::vector<std::vector<double>> RealsenseInfo1::getBlockInfo()
{
    while(!blockisReceive)
    {
        ROS_INFO("Wait for block info!");
    }
    std::vector<std::vector<double>> blockpos_temp;
    blockpos_temp.clear();
    std::vector<double> help_temp;
    for(int i=0;i<3;i++)
    {
        if(blockpos[i][0]!=0)
        {
            for(int j=0;j<6;j++)
            {
                help_temp.push_back(blockpos[i][j]);
            }
            blockpos_temp.push_back(help_temp);
            help_temp.clear();
        }
    }
    return blockpos_temp;
}

// std::vector<std::vector<double>> RealsenseInfo1::getHeadInfo()
// {
//     while(!headisReceive)
//     {
//         ROS_INFO("Wait for head info!");
//     }
//     std::vector<std::vector<double>> headpos_temp;
//     headpos_temp.clear();
//     std::vector<double> help_temp;
//     for(int i=0;i<3;i++)
//     {
//         if(headpos[i][0]!=0)
//         {
//             for(int j=0;j<6;j++)
//             {
//                 help_temp.push_back(blockpos[i][j]);
//             }
//             headpos_temp.push_back(help_temp);
//             help_temp.clear();
//         }
//     }
//     return headpos_temp;
// }

std::vector<double> RealsenseInfo1::getBoxInfo()
{
    while(!boxisReceive)
    {
        ROS_INFO("Wait For box");
    }
    std::cout<<"Get Box!"<<std::endl;
    std::vector<double> result;
    result.clear();
    if(boxpos[0]!=0)
    {
        for(int i=0;i<6;i++)
        {
            result.push_back(boxpos[i]);
        }
    }
    std::cout<<"result"<<std::endl;
    for(auto it=result.begin();it!=result.end();it++)
    {
        std::cout<<*it<<std::endl;
    }
    return result;

}
std::vector<double> RealsenseInfo1::getLeftCompensationInfo()
{
    std::vector<double> compen_l;
    compen_l.resize(3);
    compen_l.clear();
    if(this->method==0)
    {
        for (int i = 0; i < 3; ++i)
            compen_l[i] = this->left_compensation[i]; // 角度, x方向, y方向
    }
    // else
    // {
    //     compen_l[0]=wrist_cattle[0];
    //     compen_l[1]=wrist_cattle[1];
    //     compen_l[2]=atan2(wrist_bottom[0]-wrist_cattle[0],wrist_bottom[1]-wrist_cattle[1])*180/PI;
    // }
    return compen_l;
}

std::vector<double> RealsenseInfo1::getRightCompensationInfo()
{
    std::vector<double> compen_r;
    compen_r.resize(3);
    compen_r.clear();
    for (int i = 0; i < 3; ++i)
        compen_r[i] = this->right_compensation[i]; // 角度, x方向,y方向
    return compen_r;
}

// int RealsenseInfo1::getWidthInfo()
// {
//     return box_width;
// }