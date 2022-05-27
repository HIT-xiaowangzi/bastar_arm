//Created by wangzirui on 2022/2/24
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "baxtercontroller.h"
#include "Realsense_info2.h"
#include <thread>
#include <future>

#define L_ARM "left"
#define R_ARM "right"

const double upDistance=0.11;//上方的一个余量
const double x_bias=0.10;
const double y_bias=0.10;
const float DEGREE2RADIUS = 0.0174533;
const std::vector<double> target={0.8,0.2,0.01,3.14,0,3.14};//终点位置确定，此处给定
//手爪垂直向下抓位姿：3.14,0,3.14
class Task1
{
private:
    ros::NodeHandle nh;
    BaxterController* baxter;
    RealsenseInfo2* camera;
    bool isPicking;
public:
    Task1(ros::NodeHandle nh)
    {
        this->nh=nh;
        baxter=new BaxterController(nh);
        camera=new RealsenseInfo2(nh);
        isPicking=false;
    }
    ~Task1()
    {
        delete baxter;
        delete camera;
    }
    std::vector<double> getCurPos(std::string armID)
    {
        std::vector<double>currJnt=baxter->getJntPos(armID);
        std::vector<double> Cart;
        Cart.resize(6);
        Cart.clear();
        baxter->joint2Cart(currJnt,Cart,armID);
        return Cart;
    }
    void GraspOneBlock(std::vector<double> &objPos,std::vector<double> &tarPos,std::string armID)
    {
        ROS_INFO("Start Grasping!");
        isPicking=true;
        std::vector<double> objPos_temp,objPos_temp2;
        std::vector<double> tarPos_temp;
        std::vector<std::vector<double>> path;
        path.clear();
        path.push_back(getCurPos(armID));
        objPos_temp.resize(6);
        objPos_temp.clear();
        tarPos_temp.resize(6);
        tarPos_temp.clear();
        //ROS_INFO("1");
        std::cout<<objPos.at(1);
        //ROS_INFO("3");
        for(int i=0;i<6;i++)
        {
            objPos_temp.push_back(objPos.at(i));
            objPos_temp2.push_back(objPos.at(i));
        }
        objPos_temp.at(2)+=upDistance;
        //ROS_INFO("3");
        for(int i=0;i<6;i++)
        {
            tarPos_temp.push_back(tarPos.at(i));
        }
        tarPos_temp.at(2)+=upDistance;
        //ROS_INFO("2");
        path.push_back(objPos_temp);
        baxter->moveTo(path,0.02,0.01,0.25,0.1,armID);//移到目标块正上方
        path.clear();

        //开始补偿角度
        ros::Duration(1.0).sleep();
        std::vector<double> compensation;
        if(armID==L_ARM)
        {
            compensation=camera->getLeftCompensationInfo();
        }
        else
        {
            compensation=camera->getRightCompensationInfo();
        }
        // compensation[1]*=0.0005;
        // compensation[2]*=0.0005;//位置上的补偿
        // std::vector<double> posHold=objPos_temp;
        // posHold[0]+=compensation[1];
        // posHold[1]+=compensation[2];
        // path.push_back(getCurPos(armID));
        // path.push_back(posHold);
        // baxter->moveTo(path,0.02,0.01,0.1,0.07,armID);

        //调整位置
        //std::vector<double> CurPos=getCurPos(armID);
        path.clear();
        std::vector<double> CurJnt=baxter->getJntPos(armID);
        CurJnt[6]-=(compensation[0]*DEGREE2RADIUS);
        if(armID==L_ARM)
        {
            baxter->leftMoveOnce(CurJnt);
        }
        else
        {
            baxter->rightMoveOnce(CurJnt);
        }
        //完成补偿
        
        std::vector<double> CurPos;
        baxter->joint2Cart(CurJnt,CurPos,armID);
       
        path.push_back(CurPos);
        CurPos[2]-=upDistance;
        path.push_back(CurPos);
        //调整最终的抓取姿态
        baxter->moveTo(path,0.02,0.01,0.1,0.07,armID);//抓块动作要慢
        //ros::Duration(2.0).sleep();
        baxter->grip(armID);
        path.clear();
        path.push_back(getCurPos(armID));
        path.push_back(objPos_temp);
        path.push_back(tarPos_temp);
        baxter->moveTo(path,0.02,0.01,0.25,0.1,armID);//移到目标位置正上方
        path.clear();
        //ros::Duration(2.0).sleep();
        path.push_back(tarPos_temp);
        path.push_back(tarPos);
        baxter->moveTo(path,0.02,0.01,0.1,0.07,armID);
        baxter->release(armID);
        path.clear();
        path.push_back(tarPos);
        tarPos_temp[2]+=0.10;
        path.push_back(tarPos_temp);
        baxter->moveTo(path,0.02,0.01,0.25,0.1,armID);
        ROS_INFO("Successfully place one block!");
        isPicking=false;
    }
    void process()
    {
        ros::Duration(2.0).sleep();
        std::vector<std::vector<double>> objPosArray=camera->getBlockInfo();
        int size=objPosArray.size();
        ros::spinOnce();
        baxter->reSetArms(1);
        ROS_INFO("Enter the control process!");
        for(auto it=objPosArray.begin();it!=objPosArray.end();it++)
        {
            for(auto it_2=it->begin();it_2!=it->end();it_2++)
            {
                std::cout<<*it_2<<" ";
            }
            std::cout<<std::endl;
        }
        std::vector<std::vector<double>> tarPos;
        tarPos.resize(3);
        tarPos.clear();
        int i;
        for(i=0;i<3;i++)
        {
            tarPos.push_back(target);
        }
        tarPos[0][0]-=x_bias;
        tarPos[0][1]-=y_bias;
        tarPos[1][0]-=x_bias;
        tarPos[2][1]-=y_bias;
        tarPos[2][0]-=2*x_bias;
        i=0;
        while(i<size)
        {
            if(objPosArray[i][1]<=0)
            {
                if(!isPicking)
                {
                    GraspOneBlock(objPosArray.at(i),tarPos.at(i),R_ARM);
                    i++;
                }
            }
            else
            {
                if(!isPicking)
                {
                    GraspOneBlock(objPosArray.at(i),tarPos.at(i),L_ARM);
                    i++;
                }
            }
        }
        ROS_INFO("Task One finished!");
        baxter->reSetArms(1);
    }
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"task_one_armcontrol");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();
    Task1 task1(nh);
    task1.process();
    spinner.stop();
    return 0;
}
