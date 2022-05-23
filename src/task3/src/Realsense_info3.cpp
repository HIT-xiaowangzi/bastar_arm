//
// Created by hit on 2022/2/24.
//

#include <thread>
#include <future>
#include "Realsense_info3.h"
#include <std_msgs/Int16.h>   // 很重要
#include <std_msgs/Float32MultiArray.h>   // 很重要
#include <queue>
#define L_ARM "left"
#define R_ARM "right"
const double PI=3.1415926535897;
int MinInd(int* arr,int len)//返回数组的最小下标
{
    int min_ind=0;
    int min_val=0;
    if(len>0)
    {
        min_val=arr[0];
    }
    for(int i=1;i<len;i++)
    {
        if(arr[i]<min_val)
        {
            min_ind=i;
            min_val=arr[i];
        }
    }
    return min_ind;
}
RealsenseInfo3::RealsenseInfo3(ros::NodeHandle n)
{
    this->nh=n;
    this->blueisReceive=false;
    this->boxisReceive=false;
    this->redisReceive=false;
    this->angle=90;
    blueruler_sub=this->nh.subscribe("blueruler_place",100,&RealsenseInfo3::BlueRulerCallback,this);
    box_info_sub=this->nh.subscribe("box_place",100,&RealsenseInfo3::boxInfoCallback,this);
    redruler_sub=this->nh.subscribe("redruler_place",100,&RealsenseInfo3::RedRulerCallback,this);
    lefthand_camera_info_sub=this->nh.subscribe("left_hand_camera",2,&RealsenseInfo3::leftcameraInfoCallback,this);
    righthand_camera_info_sub=this->nh.subscribe("right_hand_camera",2,&RealsenseInfo3::rightcameraInfoCallback,this);
    wrist_blueruler_sub=this->nh.subscribe("wrist_blueruler",2,&RealsenseInfo3::wristbluerulerCallback,this);
    wrist_redruler_sub=this->nh.subscribe("wrist_redruler",2,&RealsenseInfo3::wristredrulerCallback,this);
    wrist_box_sub=this->nh.subscribe("wrist_box",2,&RealsenseInfo3::wristboxCallback,this);
    angle_sub=this->nh.subscribe("ruler_angle",2,&RealsenseInfo3::AngleCallback,this);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<6;j++)
        {
            bluerulerpos[i][j]=0;
            redrulerpos[i][j]=0;
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
    ros::spinOnce();
}

RealsenseInfo3::~RealsenseInfo3()
{

}

void RealsenseInfo3::BlueRulerCallback(const geometry_msgs::PoseArrayConstPtr & blue_msg)
{
    if(blue_msg->poses.size()==0)
    {
        for(int i=0;i<2;i++)
        {
            for(int j=0;j<6;j++)
            {
                bluerulerpos[i][j]=0;//没有的时候清零，防止陷入死循环
            }
        }
        return;
    }
    if(blue_msg->poses[0].position.x>0 &&blue_msg->poses[0].position.x<2)
    {
        this->blueisReceive=true;
        for (int i = 0; i < blue_msg->poses.size(); i++)
        {
            bluerulerpos[i][0] = blue_msg->poses[i].position.x;
            bluerulerpos[i][1] = blue_msg->poses[i].position.y;
            bluerulerpos[i][2] = 0.035;
            bluerulerpos[i][3] = 3.14;
            bluerulerpos[i][4] = 0;
            bluerulerpos[i][5] = 3.14;
        }
        for(int j=blue_msg->poses.size();j<2;j++)
        {
            bluerulerpos[j][0]=0;
            bluerulerpos[j][1]=0;
            bluerulerpos[j][2]=0;
            bluerulerpos[j][3]=0;
            bluerulerpos[j][4]=0;
            bluerulerpos[j][5]=0;
        }
    }
    else
    {
        ROS_INFO("BlueRuler info isn't valid!");
    }
}

void RealsenseInfo3::boxInfoCallback(const geometry_msgs::PoseArrayConstPtr & box_msg)
{
    if(box_msg->poses.size()==0)
    {
        return;
    }
    if(box_msg->poses[0].position.x>0 &&box_msg->poses[0].position.x<2)
    {
        this->boxisReceive=true;
        boxpos[0]=box_msg->poses[0].position.x+0.105;//全部拍到前面
        boxpos[1]=box_msg->poses[0].position.y;
        boxpos[2]=0.35;//举高一点，便于补偿
        boxpos[3]=3.14;
        boxpos[4]=0;
        boxpos[5]=3.14;
            
    }
    else
    {
        ROS_INFO("Box info isn't valid!");
    }
}

void RealsenseInfo3::RedRulerCallback(const geometry_msgs::PoseArrayConstPtr & red_msg)
{
    if(red_msg->poses.size()==0)
    {
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<6;j++)
            {
                redrulerpos[i][j]=0;//没有的时候清零，防止陷入死循环
            }
        }
        return;
    }
    if(red_msg->poses[0].position.x>0 &&red_msg->poses[0].position.x<2)
    {
        this->redisReceive=true;
        for (int i = 0; i < red_msg->poses.size(); i++)
        {
            redrulerpos[i][0] = red_msg->poses[i].position.x;
            redrulerpos[i][1] = red_msg->poses[i].position.y;
            redrulerpos[i][2] = 0.035;
            redrulerpos[i][3] = 3.14;
            redrulerpos[i][4] = 0;
            redrulerpos[i][5] = 3.14;
        }
        for(int j=red_msg->poses.size();j<2;j++)
        {
            redrulerpos[j][0]=0;
            redrulerpos[j][1]=0;
            redrulerpos[j][2]=0;
            redrulerpos[j][3]=0;
            redrulerpos[j][4]=0;
            redrulerpos[j][5]=0;
        }
    }
    else
    {
        ROS_INFO("Red info isn't valid!");
    }
}

void RealsenseInfo3::leftcameraInfoCallback(const std_msgs::Float32MultiArrayPtr &left_camera_msg)
{
    left_compensation[0] = left_camera_msg->data.at(2);        // 角度补偿
    left_compensation[1] = left_camera_msg->data.at(0);       // x补偿
    left_compensation[2] = left_camera_msg->data.at(1);;      // y补偿
}

void RealsenseInfo3::rightcameraInfoCallback(const std_msgs::Float32MultiArrayPtr &right_camera_msg)
{
    right_compensation[0] = right_camera_msg->data.at(2);     // 角度补偿
    right_compensation[1] = right_camera_msg->data.at(0);    // x补偿
    right_compensation[2] = right_camera_msg->data.at(1);    // y补偿
}

// void RealsenseInfo3::boxWidthCallback(const std_msgs::Int16ConstPtr width_msg)
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

void RealsenseInfo3::wristbluerulerCallback(const task3::MyListArrayConstPtr &wrist_ruler_msg)
{
    int size=wrist_ruler_msg->MyLists.size();
    if(size==0)
    {
        wrist_blueruler[0]=0;
        wrist_blueruler[1]=0;
        ratio_blue=0;
        return;
    }
    else
    {
        int* dist=new int[size];
        for(int i=0;i<size;i++)
        {
            dist[i]=pow((wrist_ruler_msg->MyLists[i].x-650),2)+pow((wrist_ruler_msg->MyLists[i].y-370),2);
        }
        int min_ind=MinInd(dist,size);
        wrist_blueruler[0]=wrist_ruler_msg->MyLists[min_ind].x;
        wrist_blueruler[1]=wrist_ruler_msg->MyLists[min_ind].y;
        ratio_blue=(double)wrist_ruler_msg->MyLists[min_ind].w/(double)wrist_ruler_msg->MyLists[min_ind].h;
        delete dist; 
    }
    
}
void RealsenseInfo3::wristredrulerCallback(const task3::MyListArrayConstPtr &wrist_ruler_msg)
{
    int size=wrist_ruler_msg->MyLists.size();
    if(size==0)
    {
        wrist_redruler[0]=0;
        wrist_redruler[1]=0;
        ratio_red=0;
        return;
    }
    else
    {
        int *dist2=new int[size];
        for(int i=0;i<size;i++)
        {
            dist2[i]=pow((wrist_ruler_msg->MyLists[i].x-650),2)+pow((wrist_ruler_msg->MyLists[i].y-420),2);
        }
        int min_ind=MinInd(dist2,size);
        wrist_redruler[0]=wrist_ruler_msg->MyLists[min_ind].x;
        wrist_redruler[1]=wrist_ruler_msg->MyLists[min_ind].y;
        ratio_red=(double)wrist_ruler_msg->MyLists[min_ind].w/(double)wrist_ruler_msg->MyLists[min_ind].h;
        delete dist2;
    }
}

void RealsenseInfo3::wristboxCallback(const task3::MyListArrayConstPtr &wrist_box_msg)
{
    int size=wrist_box_msg->MyLists.size();
    if(size==0)
    {
        return;
    }
    else
    {
        int *dist3=new int[size];
        for(int i=0;i<size;i++)
        {
            dist3[i]=pow((wrist_box_msg->MyLists[i].x-650),2)+pow((wrist_box_msg->MyLists[i].y-420),2);//防止腕部相机识别出多个箱子
        }
        int min_ind=MinInd(dist3,size);
        wrist_box[0]=wrist_box_msg->MyLists[min_ind].x;
        wrist_box[1]=wrist_box_msg->MyLists[min_ind].y;
        delete dist3;
    }
}

void RealsenseInfo3::AngleCallback(const std_msgs::Float32ConstPtr &msg)
{
    if(msg==NULL && msg->data==0)
    {
        return;
    }
    this->angle=msg->data;
}
std::vector<std::vector<double>> RealsenseInfo3::getBlueRulerInfo()
{
    if(!blueisReceive)
    {
        ROS_INFO("Can't get blueruler info!");
        std::vector<std::vector<double>> temp;
        temp.clear();
        return temp;
    }
    std::vector<std::vector<double>> bluepos_temp;
    bluepos_temp.clear();
    std::vector<double> help_temp;
    for(int i=0;i<2;i++)
    {
        if(bluerulerpos[i][0]!=0)
        {
            for(int j=0;j<6;j++)
            {
                help_temp.push_back(bluerulerpos[i][j]);
            }
            bluepos_temp.push_back(help_temp);
            help_temp.clear();
        }
    }
    return bluepos_temp;
}

std::vector<std::vector<double>> RealsenseInfo3::getRedRulerInfo()
{
    if(!redisReceive)
    {
        ROS_INFO("Can't get redruler info!");
        std::vector<std::vector<double>> temp;
        temp.clear();
        return temp;
    }
    std::vector<std::vector<double>> redpos_temp;
    redpos_temp.clear();
    std::vector<double> help_temp;
    for(int i=0;i<2;i++)
    {
        if(redrulerpos[i][0]!=0)
        {
            for(int j=0;j<6;j++)
            {
                help_temp.push_back(redrulerpos[i][j]);
            }
            redpos_temp.push_back(help_temp);
            help_temp.clear();
        }
    }
    return redpos_temp;
}

std::vector<double> RealsenseInfo3::getBoxInfo()
{
    if(!boxisReceive)
    {
        ROS_INFO("Can't get box info!");
        std::vector<double> temp;
        temp.clear();
        return temp;
    }
    std::vector<double> result;
    result.clear();
    if(boxpos[0]!=0)
    {
        for(int i=0;i<6;i++)
        {
            result.push_back(boxpos[i]);
        }
    }
    return result;

}
double RealsenseInfo3::getAngle()
{
    return this->angle;
}
std::vector<double> RealsenseInfo3::getBlueCompensationInfo()
{
    std::vector<double> compen_l;
    compen_l.resize(3);
    compen_l.clear();
    compen_l[1]=wrist_blueruler[0]-650;
    compen_l[2]=wrist_blueruler[1]-420;
    compen_l[0]=ratio_blue;//蓝尺的宽高比
    return compen_l;
}

std::vector<double> RealsenseInfo3::getRedCompensationInfo()
{
    std::vector<double> compen_l;
    compen_l.resize(3);
    compen_l.clear();
    compen_l[1]=wrist_redruler[0]-650;
    compen_l[2]=wrist_redruler[1]-420;
    compen_l[0]=ratio_red;//蓝尺的宽高比
    return compen_l;
}

std::vector<double> RealsenseInfo3::getRightCompensationInfo()
{
    std::vector<double> compen_r;
    compen_r.resize(3);
    compen_r.clear();
    compen_r[1]=wrist_box[0]-650;
    compen_r[2]=wrist_box[1]-420;
    compen_r[0]=0;
    return compen_r;
}