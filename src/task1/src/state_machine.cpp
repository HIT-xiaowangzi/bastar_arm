//Created by wangzirui on 2022/3/1
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "baxtercontroller.h"
#include "Realsense_info1.h"
#include <thread>
#include <future>

#define L_ARM "left"
#define R_ARM "right"


const double upDistance1=0.13;//上方的一个余量
const double upDistance2=0.35;
const double x_bias=0.10;
const double y_bias=0.10;
const float DEGREE2RADIUS = 0.0174533;
//const double target[3][6]={{0.7,0,0.01,3.14,0,3.14},
                            //{0.7,0.1,0.01,3.14,0,3.14},{0.6,0.0,0.01,3.14,0,3.14}};//终点位置确定，此处给定
const float threshould=0.02;


class State_Machine
{
private:
    ros::NodeHandle nh;
    BaxterController* baxter;
    RealsenseInfo1* camera;
    std::vector<double>init_center;
    bool isFirstMsg;
    std::vector<double> boxPos;//箱子中心
    std::vector<std::vector<double>> TarPos;
    //bool isPicking;
public:
    State_Machine(ros::NodeHandle nh)
    {
        this->nh=nh;
        baxter=new BaxterController(nh);
        ROS_INFO("baxter!");
        camera=new RealsenseInfo1(nh);
        ROS_INFO("camera!");
        init_center.resize(3);
        init_center.clear();
        isFirstMsg=true;
        boxPos.resize(6);
        boxPos.clear();
        TarPos.resize(3);
        TarPos.clear();
        //isPicking=false;
    }

    ~State_Machine()
    {
        delete baxter;
        delete camera;
    }

    void getBoxPos(std::string armID)
    {
        baxter->reSetArms(8);
        std::vector<double> boxPos_init=camera->getBoxInfo();
        ros::Duration(1.0).sleep();
        //ROS_INFO("g!");
        // std::vector<std::vector<double>> path;
        // path.clear();
        // path.push_back(getCurPos(armID));
        // path.push_back(boxPos_init);
        // baxter->moveTo(path,0.02,0.01,0.25,0.1,armID);
        // std::vector<double> compensation;
        // if(armID==L_ARM)
        // {
        //     compensation=camera->getLeftCompensationInfo();
        // }
        // else
        // {
        //     compensation=camera->getRightCompensationInfo();
        // }

        // double threshould_1=30.00,threshould_2=2000.00,ratio_k=3;
        // while (pow(compensation[1],2)+pow(compensation[2],2)>threshould_1)
        // {
        //     path.clear();
        //     if(armID==L_ARM)
        //     {
        //         compensation=camera->getLeftCompensationInfo();
        //     }
        //     else
        //     {
        //         compensation=camera->getRightCompensationInfo();
        //     }
        //     //move
        //     std::vector<double> PosCur_2=getCurPos(armID);
        //     path.push_back(PosCur_2);
        //     std::cout<<"补偿之前"<<std::endl;
        //     for(auto it=PosCur_2.begin();it!=PosCur_2.end();it++)
        //     {
        //         std::cout<<*it<<" ";
        //     }
        //     std::cout<<std::endl;
        //     double x_move=double(compensation[1])/(1600*ratio_k);
        //     double y_move=double(compensation[2])/(1600*ratio_k);
        //     PosCur_2[1]-=x_move;
        //     PosCur_2[0]-=y_move;
        //     std::cout<<"补偿之后"<<std::endl;
        //     for(auto it=PosCur_2.begin();it!=PosCur_2.end();it++)
        //     {
        //         std::cout<<*it<<" ";
        //     }
        //     std::cout<<std::endl;
        //     path.push_back(PosCur_2);
        //     baxter->moveTo(path,0.02,0.01,0.1,0.07,armID);
        //     std::cout<<"移动之后"<<std::endl;
        //     std::vector<double> temp=getCurPos(armID);
        //     for(auto it=temp.begin();it!=temp.end();it++)
        //     {
        //         std::cout<<*it<<" ";
        //     }
        //     std::cout<<std::endl;
        //     //delay
        //     ros::Duration(0.5).sleep();
        // } 
        boxPos=boxPos_init;
        int i=0;
        for(i=0;i<6;i++)
        {
            TarPos[1].push_back(boxPos[i]);
        }
        TarPos[1][0]-=0.02;
        TarPos[1][1]+=0.03;
        TarPos[1][2]=0.07;
        for(i=0;i<6;i++)
        {
            TarPos[0].push_back(boxPos[i]);
        }
        TarPos[0][0]-=0.02;
        TarPos[0][1]=TarPos[1][1]+0.08;
        TarPos[0][2]=0.07;
        for(i=0;i<6;i++)
        {
            TarPos[2].push_back(boxPos[i]);
        }
        TarPos[2][0]-=0.02;
        TarPos[2][1]=TarPos[1][1]-0.08;
        TarPos[2][2]=0.07;
        std::cout<<"Box Position:"<<std::endl;
        for(auto it=boxPos.begin();it!=boxPos.end();it++)
        {
            std::cout<<*it<<" ";
        }
        std::cout<<std::endl;
    }
    std::vector<double> Camera_init()
    {
        while(isFirstMsg)
        {
            std::vector<std::vector<double>> first_msg=camera->getBlockInfo();
            double x_sum,y_sum,z_sum;
            x_sum=0;
            y_sum=0;
            z_sum=0;
            int size=first_msg.size();
            if(size!=0)
            {
                for(int i=0;i<size;i++)
                {
                    x_sum+=first_msg[i][0];
                    y_sum+=first_msg[i][1];
                    z_sum+=first_msg[i][2];
                }
                double x_center=x_sum/size;
                double y_center=y_sum/size;
                double z_center=z_sum/size;
                std::vector<double> ans;
                ans.clear();
                ans.push_back(x_center);
                ans.push_back(y_center);
                ans.push_back(z_center);
                ROS_INFO("The initial center is : %lf,%lf,%lf",x_center,y_center,z_center);
                isFirstMsg=false;
                return ans;
            }
            else
            {
                ROS_ERROR("Can't find the initial center!");
            }
        }
    }

    std::vector<std::vector<double>> GetValidBlocks()//获得真正有效的块的信息
    {
        std::vector<std::vector<double>> objPos=camera->getBlockInfo();
        std::vector<std::vector<double>> ValidBlocks;
        ValidBlocks.clear();
        int size=objPos.size();
        for(int i=0;i<size;i++)
        {
            if(pow((objPos[i][0]-init_center[0]),2)+pow((objPos[i][1]-init_center[1]),2)+
            pow((objPos[i][2]-init_center[2]),2)<threshould)//距离中心点足够近，是有效的块
            {
                ValidBlocks.push_back(objPos[i]);
                std::cout<<objPos[i][0]<<" "<<objPos[i][1]<<" "<<objPos[i][2]<<" "<<objPos[i][3]<<" "
                <<objPos[i][4]<<" "<<objPos[i][5]<<" "<<std::endl;
            }
        }
        return ValidBlocks;

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

    void GraspOneBlock(std::vector<double> &objPos,std::vector<double> &tarPos,std::string armID)//抓一个块
    {
        ROS_INFO("Start Grasping!");
        //isPicking=true;
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
            //objPos_temp2.push_back(objPos.at(i));
        }
        objPos_temp.at(2)+=upDistance1;
        //ROS_INFO("3");
        for(int i=0;i<6;i++)
        {
            tarPos_temp.push_back(tarPos.at(i));
        }
        tarPos_temp.at(2)+=upDistance2;
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


        //c[1]=
        double threshould_1=30.00,threshould_2=2000.00,ratio_k=3;
        //1.位置粗略补偿
        // double four_ways[4][2]={{-1,0},{0,-1},{1,0},{0,1}};
        // int count=0;
        // std::vector<double> PosCur_1=getCurPos(armID);
        // int radius=0.03;
        // while(pow(compensation[1],2)+pow(compensation[2],2)>threshould_2)
        // {
        //     int index=count%4;
        //     int times=count/4+1;
        //     std::vector<double> PosCur1_copy=PosCur_1;
        //     path.clear();
        //     path.push_back(getCurPos(armID));
        //     PosCur1_copy[0]+=four_ways[index][0]*radius*times;
        //     PosCur1_copy[1]+=four_ways[index][1]*radius*times;
        //     path.push_back(PosCur1_copy);
        //     baxter->moveTo(path,0.02,0.01,0.1,0.07,armID);
        //     ros::Duration(1).sleep();
        //     count++;    
        //     if(armID==L_ARM)
        //     {
        //         compensation=camera->getLeftCompensationInfo();
        //     }
        //     else
        //     {
        //         compensation=camera->getRightCompensationInfo();
        //     }
        // }
        //2.位置精确补偿
        while (pow(compensation[1],2)+pow(compensation[2],2)>threshould_1)
        {
            path.clear();
            if(armID==L_ARM)
            {
                compensation=camera->getLeftCompensationInfo();
            }
            else
            {
                compensation=camera->getRightCompensationInfo();
            }
            //move
            std::vector<double> PosCur_2=getCurPos(armID);
            path.push_back(PosCur_2);
            std::cout<<"补偿之前"<<std::endl;
            for(auto it=PosCur_2.begin();it!=PosCur_2.end();it++)
            {
                std::cout<<*it<<" ";
            }
            std::cout<<std::endl;
            double x_move=double(compensation[1])/(1600*ratio_k);
            double y_move=double(compensation[2])/(1600*ratio_k);
            PosCur_2[1]-=x_move;
            PosCur_2[0]-=y_move;
            std::cout<<"补偿之后"<<std::endl;
            for(auto it=PosCur_2.begin();it!=PosCur_2.end();it++)
            {
                std::cout<<*it<<" ";
            }
            std::cout<<std::endl;
            path.push_back(PosCur_2);
            baxter->moveTo(path,0.02,0.01,0.1,0.07,armID);
            std::cout<<"移动之后"<<std::endl;
            std::vector<double> temp=getCurPos(armID);
            for(auto it=temp.begin();it!=temp.end();it++)
            {
                std::cout<<*it<<" ";
            }
            std::cout<<std::endl;
            //delay
            ros::Duration(0.5).sleep();
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
        CurPos[2]-=upDistance1;
        path.push_back(CurPos);
        //调整最终的抓取姿态
        baxter->moveTo(path,0.02,0.01,0.1,0.07,armID);//抓块动作要慢
        ros::Duration(1.0).sleep();
        baxter->grip(armID);
        path.clear();
        path.push_back(getCurPos(armID));
        objPos_temp[2]=objPos[2]+upDistance2;
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
        path.push_back(objPos_temp);
        baxter->moveTo(path,0.02,0.01,0.25,0.1,armID);
        ROS_INFO("Successfully place one block!");
        //isPicking=false;
    }

    
    void state(int size,std::vector<std::vector<double>> ValidBlocks)
    {
        int index=3-size;
        std::vector<double> tarPos;
        tarPos.clear();
        for(int i=0;i<6;i++)
        {
            tarPos.push_back(TarPos[index][i]);
        }
        std::vector<double> objPos;
        objPos.clear();
        objPos=ValidBlocks[0];
        if(index==0)
        {
            objPos[2]=0.1075;//第一个块改一下高度
        }
        if(objPos[1]>=0)
        {
            GraspOneBlock(objPos,tarPos,L_ARM);
        }
        else
        {
            GraspOneBlock(objPos,tarPos,R_ARM);
        }
    }

    void process()
    {
        ros::Duration(2.0).sleep();
        init_center=Camera_init();
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        std::vector<std::vector<double>> ValidBlocks;
        ValidBlocks.clear();
        ValidBlocks=GetValidBlocks();
        int size;
        size=ValidBlocks.size();
        ros::spinOnce();
        baxter->reSetArms(8);
        while(size!=0)
        {
            ROS_INFO("size:[%d]",size);
            state(size,ValidBlocks);
            ros::Duration(1.0).sleep();
            ValidBlocks.clear();
            ValidBlocks=GetValidBlocks();
            size=ValidBlocks.size();
            ros::spinOnce();
        }
        ROS_INFO("Task One finished!");
        baxter->reSetArms(8);
    }
    void test()
    {
        baxter->reSetArms(1);
    }
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"task_one_armcontrol");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();
    State_Machine task1(nh);
    // task1.test();
    // // task1.getBoxPos(R_ARM);
    // std::cout<<"1"<<std::endl;
    // task1.getBoxPos(R_ARM);
    // ros::Duration(2.0).sleep();
    // task1.process();
    // std::vector<double> CurPosLeft,CurPosRight;
    // CurPosLeft=task1.getCurPos(L_ARM);
    // CurPosRight=task1.getCurPos(R_ARM);
    // std::cout<<"Left:"<<std::endl;
    // for(auto it=CurPosLeft.begin();it!=CurPosLeft.end();it++)
    // {
    //     std::cout<<*it<<" ";
    // }
    // std::cout<<std::endl;
    // std::cout<<"Right:"<<std::endl;
    // for(auto it=CurPosRight.begin();it!=CurPosRight.end();it++)
    // {
    //     std::cout<<*it<<" ";
    // }
    // std::cout<<std::endl;
    spinner.stop();
    return 0;
}