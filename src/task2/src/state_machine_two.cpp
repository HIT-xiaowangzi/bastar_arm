//Created by wangzirui on 2022/3/1
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "baxtercontroller.h"
#include "Realsense_info2.h"
#include <thread>
#include <future>
#include <math.h>
#include "elbowcontroller.h"
#include <string.h>
#define L_ARM "left"
#define R_ARM "right"

const double PI=3.1415926535897;
const double upDistance1=0.27;//上方的一个余量
const double upDistance2=0.45;//箱子高度
const double x_bias=0.10;
const double y_bias=0.10;
const float DEGREE2RADIUS = 0.0174533;
//const double target[3][6]={{0.7,0.1,0.01,3.14,0,3.14},
//                           {0.7,0.2,0.01,3.14,0,3.14},{0.6,0.1,0.01,3.14,0,3.14}};//终点位置确定，此处给定
const float threshould=0.04;

const double left_change[6]={0.73,0.05,0.5,-1.57,0,3.14};//左手换手坐标

const double right_change[6]={0.705,-0.13,0.50,1.57,0,-3.14};//右手换手坐标

const double homePosLeft[7]  = { 0.1, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50};
const double homePosRight[7] = {-0.1, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50};
const double objPos[6]={0.742363,0.344003,-0.0604307,-3.07215,-0.00242124,-2.81699};
const double tarPos[6]={0.8217,-0.37414,-0.0292459,3.13452,-0.071637,-2.8268};

const double a=-322.7486276;
const double b=676.2925666;
bool first_flag=true;
//箱子完整状态下在图中的宽度
class State_Machine
{
private:
    ros::NodeHandle nh;
    BaxterController* baxter;
    RealsenseInfo2* camera;
    ElbowController* elbow;
    std::vector<double>init_center;
    std::vector<std::vector<double>> TarGet;
    //std::vector<double> boxPos;
    bool isFirstMsg;
    int theta_method;//获取补偿角度的方法 法1：模板匹配（0)法2：Realsense-yolo(1) 法3：腕部相机-yolo(2)
    //备注：模板匹配方法z轴高度0.4m,腕部相机-yolo方法z轴高度0.3m
    //bool yolotheta=false;//false:模板匹配确定theta true:yolo确定theta
    //bool isPicking;
    int flag=0;//左手相机标志位
public:
    State_Machine(ros::NodeHandle nh)
    {
        this->nh=nh;
        baxter=new BaxterController(nh);
        camera=new RealsenseInfo2(nh);
        elbow=new ElbowController(nh);
        init_center.resize(3);
        init_center.clear();
        //boxPos.resize(3);
        //boxPos.clear();
        isFirstMsg=true;
        //isPicking=false;
        TarGet.resize(3);
        TarGet.clear();
        this->theta_method=2;//第二种方法
        baxter->reSetArms(8);
    }

    ~State_Machine()
    {
        delete baxter;
        delete camera;
        delete elbow;
    }
    std::vector<double> getBoxPos()
    {
        ros::Duration(2.0).sleep();
        std::cout<<"进！";
        std::vector<double> boxPos_init=camera->getBoxInfo();
        std::vector<double> first_end,middle_point;
        first_end.resize(6);
        first_end.clear();
        middle_point.resize(6);
        middle_point.clear();
        for(auto it=boxPos_init.begin();it!=boxPos_init.end();it++)
        {

            first_end.push_back(*it);
        }
        first_end.push_back(3.14);
        first_end.push_back(0);
        first_end.push_back(3.14);
        middle_point=first_end;
        middle_point[0]+=0.10;
        middle_point[1]-=0.15;
        middle_point[2]+=0.15;
        std::vector<std::vector<double>> path;
        path.clear();
        path.push_back(getCurPos(R_ARM));
        path.push_back(middle_point);
        path.push_back(first_end);
        baxter->moveTo(path,0.02,0.01,0.1,0.07,R_ARM);
         //开始补偿
        //ros::Duration(0.5).sleep();
        std::vector<double> compensation;
        compensation=camera->getRightCompensationInfo();
        auto startTime = std::chrono::high_resolution_clock::now();
        double threshould_1=100.00,threshould_2=2000.00,ratio_k=4;
        //1.位置补偿
        //ros::Duration(1.0).sleep();
        while (pow(compensation[1],2)+pow(compensation[2],2)>threshould_1)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            auto endTime=std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::ratio<1, 1> > duration_s(endTime - startTime);
            path.clear();
            compensation=camera->getRightCompensationInfo();
            //move
            std::vector<double> PosCur_2=getCurPos(R_ARM);
            path.push_back(PosCur_2);
            std::cout<<std::endl;
            double x_move=double(compensation[1])/(1600*ratio_k);
            double y_move=double(compensation[2])/(1600*ratio_k);
            PosCur_2[1]-=x_move;
            PosCur_2[0]-=y_move;
            PosCur_2[2]=first_end[2];
            PosCur_2[3]=first_end[3];
            PosCur_2[4]=first_end[4];
            PosCur_2[5]=first_end[5];
            path.push_back(PosCur_2);
            baxter->moveTo(path,0.02,0.01,0.07,0.05,R_ARM);
            //delay
            ros::Duration(0.3).sleep();
            if(duration_s.count()>8)
            {
                std::cout<<"Compensation timeout!"<<std::endl;
                break;
            }
        } 
        std::vector<double> final_end=getCurPos(R_ARM);
        std::cout<<"最终箱子坐标"<<std::endl;
        for(auto it=final_end.begin();it!=final_end.end();it++)
        {

            std::cout<<*it<<" "<<std::endl;
        }

        for(int i=0;i<6;i++)
        {
            TarGet[0].push_back(final_end[i]);
            TarGet[1].push_back(final_end[i]);
            TarGet[2].push_back(final_end[i]);
        }
        //TarGet[0][0]-=0.03;
        TarGet[0][1]=TarGet[0][1]-0.08;//左偏
        TarGet[0][2]=0.04;
        TarGet[0][3]=-3.14;
        TarGet[0][5]=-1.57;
        //TarGet[1][0]-=0.03;
        TarGet[1][1]=TarGet[1][1]+0.08;//右偏
        TarGet[1][2]=0.04;
        TarGet[1][3]=-3.14;
        TarGet[1][5]=-1.57;
        //TarGet[2][0]-=0.03;
        TarGet[2][2]=0.21;
        TarGet[2][3]=-3.14;
        TarGet[2][5]=-1.57;
        ros::Duration(15.0).sleep();
        std::vector<double>CurPos=final_end;
        std::vector<double>CurPos2=final_end;
        CurPos2[0]+=0.10;
        CurPos2[1]-=0.05;
        CurPos2[2]+=0.10;
        path.clear();
        path.push_back(CurPos);
        path.push_back(CurPos2);
        baxter->moveTo(path,0.02,0.01,0.07,0.05,R_ARM);
        return final_end;
    }

    double dist(std::vector<double> a,std::vector<double> b)
    {
        return pow((a[0]-b[0]),2)+pow((a[1]-b[1]),2)+pow((a[2]-b[2]),2);
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
        std::cout<<"Valid!";
        std::vector<std::vector<double>> objPos=camera->getBlockInfo();
        std::vector<std::vector<double>> ValidBlocks;
        ValidBlocks.clear();
        int size=objPos.size();
        for(int i=0;i<size;i++)
        {
            // if(pow((objPos[i][0]-init_center[0]),2)+pow((objPos[i][1]-init_center[1]),2)+
            // pow((objPos[i][2]-init_center[2]),2)<threshould)//距离中心点足够近，是有效的块
            // {
            ValidBlocks.push_back(objPos[i]);
            // }
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
    std::vector<double> elbowCurPos(std::string armID)
    {
        std::vector<double>currJnt=elbow->getJntPos(armID);
        std::vector<double> Cart;
        Cart.resize(6);
        Cart.clear();
        elbow->joint2Cart(currJnt,Cart,armID);
        return Cart;
    }

    void GraspCattle(std::vector<double> &objPos,std::vector<double> &tarPos,std::string armID)//抓一个块
    {
        baxter->reSetArms(8);
        std::string OtherarmID;
        if(armID==L_ARM)
        {
            OtherarmID=R_ARM;
        }
        else
        {
            OtherarmID=L_ARM;
        }
        std::vector<double> left_change_pos;
        std::vector<double> right_change_pos;
        left_change_pos.resize(6);
        left_change_pos.clear();
        right_change_pos.resize(6);
        right_change_pos.clear();

        for(int i=0;i<6;i++)
        {
            left_change_pos.push_back(left_change[i]);
        }

        for(int j=0;j<6;j++)
        {
            right_change_pos.push_back(right_change[j]);
        }
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
        //std::cout<<objPos.at(1);
        //ROS_INFO("3");
        for(int i=0;i<6;i++)
        {
            objPos_temp.push_back(objPos.at(i));
            objPos_temp2.push_back(objPos.at(i));
        }
        objPos_temp.at(2)+=upDistance1;
        //ROS_INFO("3");
        for(int i=0;i<6;i++)
        {
            tarPos_temp.push_back(tarPos.at(i));
        }
        tarPos_temp[2]=upDistance2;
        tarPos_temp[3]=-3.14;
        tarPos_temp[4]=0;
        tarPos_temp[5]=-1.57;
        //ROS_INFO("2");
        path.push_back(objPos_temp);
        baxter->moveTo(path,0.02,0.01,0.25,0.1,armID);//移到目标块正上方
        path.clear();

        //开始补偿
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
        double threshould_1=20.00,threshould_2=2000.00,ratio_k=5;
        //1.位置补偿
        ros::Duration(1.0).sleep();
        auto startTime = std::chrono::high_resolution_clock::now();
        while (pow(compensation[1],2)+pow(compensation[2],2)>threshould_1)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            auto endTime=std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::ratio<1, 1> > duration_s(endTime - startTime);
            path.clear();
            if(armID==L_ARM)
            {
                compensation=camera->getLeftCompensationInfo();
                std::cout<<compensation[0]<<std::endl;
            }
            else
            {
                compensation=camera->getRightCompensationInfo();
            }
            //move
            std::vector<double> PosCur_2=getCurPos(armID);
            path.push_back(PosCur_2);
            std::cout<<std::endl;
            double x_move=double(compensation[1])/(1600*ratio_k);
            double y_move=double(compensation[2])/(1600*ratio_k);
            PosCur_2[1]-=x_move;
            PosCur_2[0]-=y_move;
            path.push_back(PosCur_2);
            baxter->moveTo(path,0.02,0.01,0.1,0.07,armID);
            std::cout<<std::endl;
            //delay
            ros::Duration(0.5).sleep();
            if(duration_s.count()>30)
            {
                std::cout<<"Compensation timeout!"<<std::endl;
                break;
            }
        } 

        //2.角度补偿
        double compen_theta=0;
        compen_theta=compensation[0];
        std::cout<<"准备开始转"<<std::endl;
        std::cout<<compen_theta<<std::endl;
        path.clear();
        std::vector<double> CurJnt=baxter->getJntPos(armID);
        CurJnt[6]-=(compen_theta*DEGREE2RADIUS);
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
        //ros::Duration(2.0).sleep();
        baxter->grip(armID);
        path.clear();
        path.push_back(getCurPos(armID));
        path.push_back(objPos_temp);
        baxter->moveTo(path,0.02,0.01,0.25,0.1,armID);

        //开始换手
        path.clear();
        path.push_back(getCurPos(OtherarmID));
        std::vector<double> right_change_pos_temp(right_change_pos);
        right_change_pos_temp[0]-=0.15;
        std::vector<double> left_change_pos_temp(left_change_pos);
        left_change_pos_temp[0]-=0.15;
        if(OtherarmID==R_ARM)
        {
            path.push_back(right_change_pos_temp);
            //path.push_back(right_change_pos);
        }
        else
        {
            path.push_back(left_change_pos_temp);
            //path.push_back(left_change_pos);
        }
        baxter->moveTo(path,0.02,0.01,0.25,0.1,OtherarmID);//右手到达目标位置
        path.clear();
        path.push_back(getCurPos(armID));
        if(armID==L_ARM)
        {
            path.push_back(left_change_pos);
        }
        else
        {
            path.push_back(right_change_pos);
        }
        
        baxter->moveTo(path,0.02,0.01,0.25,0.1,armID);//左手到达目标位置
        path.clear();
        //右手往前送一段，这样更准
        path.push_back(getCurPos(OtherarmID));
        path.push_back(right_change_pos);
        baxter->moveTo(path,0.02,0.01,0.10,0.1,OtherarmID);
        ros::Duration(1.0).sleep();
        baxter->grip(OtherarmID);//右手抓住
        baxter->release(armID);//左手松开
        ros::Duration(1.0).sleep();
        path.clear();
        std::vector<double> CurPos2=getCurPos(armID);
        path.push_back(CurPos2);
        CurPos2[1]+=0.125;//向x方向移20cm，防止把牛带倒
        path.push_back(CurPos2);
        CurPos2[1]+=0.125;
        path.push_back(CurPos2);
        baxter->moveTo(path,0.02,0.01,0.25,0.1,armID);
        path.clear();
        // std::vector<double> left_home(homePosLeft,homePosLeft+7);
        // std::vector<double> right_home(homePosRight,homePosRight+7);
        // if(armID==L_ARM)
        // {
        //     baxter->leftMoveOnce(left_home);//左手闪开
        // }
        // else
        // {
        //     baxter->rightMoveOnce(right_home);
        // }
        path.clear();
        //结束换手
        std::vector<double> start_point=getCurPos(OtherarmID);
        std::vector<double>middle_point;
        middle_point.resize(6);
        middle_point.clear();
        for(int i=0;i<6;i++)
        {
            middle_point.push_back(TarGet[0][i]);
        }
        //middle_point[1]-=0.03;
        middle_point[2]+=upDistance2;
        path.push_back(start_point);
        path.push_back(middle_point);
        path.push_back(tarPos_temp);
        baxter->moveTo(path,0.02,0.01,0.07,0.05,OtherarmID);//移到目标位置正上方
        path.clear();
        ros::Duration(1.0).sleep();
        path.push_back(getCurPos(OtherarmID));
        path.push_back(tarPos);
        baxter->moveTo(path,0.02,0.01,0.1,0.07,OtherarmID);
        baxter->release(OtherarmID);
        path.clear();
        ros::Duration(1.0).sleep();
        path.push_back(tarPos);
        tarPos_temp[2]+=0.40;
        path.push_back(tarPos_temp);
        baxter->moveTo(path,0.02,0.01,0.25,0.1,OtherarmID);
        ROS_INFO("Successfully place one cattle!");
        //isPicking=false;
    }

    
    void state(int size,std::vector<std::vector<double>> ValidBlocks)
    {
        int index=3-size;
        std::vector<double> tarPos;
        tarPos.clear();
        if(first_flag==true)
        {
            for(int i=0;i<6;i++)
            {
                tarPos.push_back(TarGet[0][i]);
            }
            first_flag=false;
        }
        else
        {
            for(int i=0;i<6;i++)
            {
                tarPos.push_back(TarGet[index][i]);
            }
        }
        std::vector<double> objPos;
        objPos.clear();
        objPos=ValidBlocks[0];
        GraspCattle(objPos,tarPos,L_ARM);
    }

    void process()
    {
        std::vector<double> dcw=getBoxPos();
        ros::Duration(2.0).sleep();
        init_center=Camera_init();
        ros::Duration(1.0).sleep();
        std::cout<<"1"<<std::endl;
        std::vector<std::vector<double>> ValidBlocks;
        ValidBlocks.clear();
        ValidBlocks=GetValidBlocks();
        std::cout<<"2"<<std::endl;
        int size;
        size=ValidBlocks.size();
        ROS_INFO("size:[%d]",size);
        std::cout<<"3"<<std::endl;
        ros::spinOnce();
        std::cout<<"4"<<std::endl;
        baxter->reSetArms(8);
        while(size!=0)
        {
            state(size,ValidBlocks);
            ros::Duration(1.0).sleep();
            ValidBlocks.clear();
            ValidBlocks=GetValidBlocks();
            size=ValidBlocks.size();
            ros::spinOnce();
        }
        baxter->reSetArms(8);
        ros::Duration(5.0).sleep();
        box_process(dcw);
        ROS_INFO("Task Two finished!");
        //exit(0);
    }
    
    //左手姿态：-1.57,0,0
    //右手姿态：-1.57,0,0
    // void test()
    // {
    //     std::cout<<"Start!";
    //     baxter->reSetArms(1);
    //     //std::cout<<"2";
    //     std::vector<std::vector<double>> path;
    //     path.clear();
    //     path.push_back(getCurPos(L_ARM));
    //     //std::cout<<"1";
    //     //std::vector<double> dst=getBoxPos();
    //     //std::vector<double> dst(objPos,objPos+6);
    //     double dst[6]={0.8,0.2,0.30,3.14,0,3.14};
    //     std::vector<double> dst1(dst,dst+6);
    //     path.push_back(dst1);
    //     baxter->moveTo(path,0.02,0.01,0.25,0.1,L_ARM);
    // }
    void box_process(std::vector<double> dcw)
    {
        //std::vector<double> dcw=getBoxPos();
        std::vector<double> left_elbow_pos=dcw;
        std::vector<double> right_elbow_pos=dcw;
        baxter->reSetArms(8);
        ros::Duration(2.0).sleep();
        left_elbow_pos[1]+=0.33;
        left_elbow_pos[2]=0.05;
        left_elbow_pos[3]=-1.57;
        left_elbow_pos[4]=0;
        left_elbow_pos[5]=0;
        right_elbow_pos[1]-=0.37;
        right_elbow_pos[2]=0.04;
        right_elbow_pos[3]=-1.57;
        right_elbow_pos[4]=0;
        right_elbow_pos[5]=0;
        test_box(left_elbow_pos,right_elbow_pos);
        ros::Duration(2.0).sleep();
        Release_Test();

    }
    void test_box(std::vector<double> init_left,std::vector<double> init_right)
    {
        auto thd1 = std::async(std::launch::async, &State_Machine::left_move,this,init_left);
        auto thd2 = std::async(std::launch::async, &State_Machine::right_move, this,init_right);
        thd1.wait();
        thd2.wait();
        thd1 = std::async(std::launch::async, &State_Machine::test_left,this);
        thd2 = std::async(std::launch::async, &State_Machine::test_right, this);
        thd1.wait();
        thd2.wait();
    }
    void left_move(std::vector<double> init_left)
    {
        std::vector<double>G=init_left;
        std::vector<std::vector<double>> path;
        path.clear();
        path.push_back(elbowCurPos(L_ARM));
        path.push_back(init_left);
        elbow->moveTo(path,0.02,0.01,0.07,0.05,L_ARM);
    }
    void right_move(std::vector<double> init_right)
    {
        std::vector<double>G=init_right;
        std::vector<std::vector<double>> path;
        path.clear();
        path.push_back(elbowCurPos(R_ARM));
        path.push_back(init_right);
        elbow->moveTo(path,0.02,0.01,0.07,0.05,R_ARM);
    }
    void left_leave()
    {
        std::vector<double>G=elbowCurPos(L_ARM);
        std::vector<double>G1=G;
        std::vector<std::vector<double>> path;
        path.clear();
        path.push_back(G);
        G1[1]+=0.06;
        path.push_back(G1);
        elbow->moveTo(path,0.02,0.01,0.07,0.05,L_ARM);
    }
    void right_leave()
    {
        std::vector<double>G=elbowCurPos(R_ARM);
        std::vector<double>G1=G;
        G1[1]-=0.06;
        std::vector<std::vector<double>> path;
        path.clear();
        path.push_back(G);
        path.push_back(G1);
        elbow->moveTo(path,0.02,0.01,0.07,0.05,R_ARM);
    }
    void test_left()
    {
        std::vector<double> G=elbowCurPos(L_ARM);
        std::vector<std::vector<double>> path;
        path.clear();
        path.push_back(G);
        std::vector<double>G1=G;
        G1[1]-=0.04;
        G1[2]=0.05;
        std::vector<double>G2=G;
        G2[1]-=0.08;
        G2[2]=0.05;
        std::vector<double> G3=G2;
        G3[2]+=0.05;
        std::vector<double> G4=G2;
        G4[2]+=0.08;
        path.push_back(G1);
        path.push_back(G2);
        elbow->moveTo(path,0.02,0.01,0.07,0.05,L_ARM);
        ros::Duration(2.0).sleep();
        path.clear();
        path.push_back(G2);
        path.push_back(G3);
        path.push_back(G4);
        elbow->moveTo(path,0.02,0.01,0.07,0.05,L_ARM);
    }
    void test_right()
    {
        std::vector<double> G=elbowCurPos(R_ARM);
        std::vector<std::vector<double>> path;
        path.clear();
        path.push_back(G);
        std::vector<double>G1=G;
        G1[1]+=0.04;
        G1[2]=0.05;
        std::vector<double>G2=G;
        G2[1]+=0.08;
        G2[2]=0.05;
        std::vector<double> G3=G2;
        G3[2]+=0.05;
        std::vector<double> G4=G2;
        G4[2]+=0.08;
        path.push_back(G1);
        path.push_back(G2);
        elbow->moveTo(path,0.02,0.01,0.07,0.05,R_ARM);
        ros::Duration(2.0).sleep();
        path.clear();
        path.push_back(G2);
        path.push_back(G3);
        path.push_back(G4);
        elbow->moveTo(path,0.02,0.01,0.07,0.05,R_ARM);

    }
    
    void test03()
    {
        std::vector<double> yeah=elbowCurPos(R_ARM);
        for(auto it=yeah.begin();it!=yeah.end();it++)
        {
            std::cout<<*it<<std::endl;
        }
    }
    void Release_Test()
    {
        std::vector<double> TorArray;
        TorArray.clear();
        //TorArray.resize(500);
        std::vector<double> LeftTor;
        int count=0;
        while(count<500)
        {
            LeftTor=baxter->getleftTor();
            //std::cout<<LeftTor[1]<<std::endl;
            if(abs(LeftTor[1])>5&& abs(LeftTor[1])<50)//排除异常值
            {
                TorArray.push_back(abs(LeftTor[1]));
                count++;
                std::cout<<"count:"<<count<<std::endl;
            }
        }
        double sum=0;
        for(int i=0;i<500;i++)
        {
            std::cout<<i<<":"<<TorArray[i]<<std::endl;
            sum=sum+TorArray[i];
        }
        double avg= sum/500;
        ROS_INFO("The average torque is: [%lf]",avg);
        ROS_INFO("Welcome to lift the box!");
        bool lift=false;
        double bias=0;
        LeftTor.clear();
        while(!lift)
        {
            LeftTor=baxter->getleftTor();
            //std::cout<<"LeftTor:"<<LeftTor[1]<<std::endl;
            if(abs(LeftTor[1])>5&& abs(LeftTor[1])<50)
            {
                bias=abs((abs(LeftTor[1])-avg)/avg);
                std::cout<<"LeftTor:"<<LeftTor[1]<<std::endl;
                std::cout<<"bias:"<<bias<<std::endl;
                if(bias>0.05)//偏差大于0.05,认为有人在托举
                {
                    lift=true;
                }
            }
        }
        ROS_INFO("I feel the touch!");
        ros::Duration(3.0).sleep();
        auto thd1 = std::async(std::launch::async, &State_Machine::left_leave,this);
        auto thd2 = std::async(std::launch::async, &State_Machine::right_leave,this);
        thd1.wait();
        thd2.wait();
        baxter->reSetArms(8);
        
    }
    void Reset()
    {
        baxter->reSetArms(1);
    }
    // void test()
    // {
    //     auto thd1 = std::async(std::launch::async, &State_Machine::process,this);
    //     auto thd2 = std::async(std::launch::async, &State_Machine::thread1, this);
    //     thd1.wait();
    //     thd2.wait();
    
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"task_two_armcontrol");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();
    State_Machine task2(nh);
    ros::Duration(2.0).sleep();
   
    //task2.test02();
    task2.process();
    //task2.test();
    spinner.stop();
    return 0;
}