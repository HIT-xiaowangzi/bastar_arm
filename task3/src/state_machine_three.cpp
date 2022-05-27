//Created by wangzirui on 2022/3/1
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "baxtercontroller.h"
#include "Realsense_info3.h"
#include <thread>
#include <future>
#include <math.h>
#include "elbowcontroller.h"
#include <string.h>
#include <map>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#define L_ARM "left"
#define R_ARM "right"
#define FILE_PATH "left.txt"
#define FILE_PATH2 "right.txt"
#define FILE_PATH3 "left_expectation.txt"
#define FILE_PATH4 "right_expectation.txt"

const double PI=3.1415926535897;
const double upDistance1=0.27;//上方的一个余量
const double upDistance2=0.41;//箱子高度
const double x_bias=0.10;
const double y_bias=0.10;
const float DEGREE2RADIUS = 0.0174533;
const float threshould=0.04;

const double homePosLeft[7]  = { 0.1, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50};
const double homePosRight[7] = {-0.1, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50};
const double objPos[6]={0.742363,0.344003,-0.0604307,-3.07215,-0.00242124,-2.81699};
const double tarPos[6]={0.8217,-0.37414,-0.0292459,3.13452,-0.071637,-2.8268};
//箱子完整状态下在图中的宽度

class State_Machine
{
private:
    ros::NodeHandle nh;
    BaxterController* baxter;
    RealsenseInfo3* camera;
    ElbowController* elbow;
    std::vector<double> TarGet;
    bool isFirstMsg;
    std::string color;
    std_msgs::String flag;
    std_msgs::String flag_ruler;
    ros::Publisher pub_box;
    ros::Publisher pub_ruler;
    bool isFirst;
    bool sync_flag;
public:
    State_Machine(ros::NodeHandle nh)
    {
        this->nh=nh;
        baxter=new BaxterController(nh);
        camera=new RealsenseInfo3(nh);
        elbow=new ElbowController(nh);
        TarGet.resize(6);
        TarGet.clear();
        baxter->reSetArms(8);
        this->color="red";
        this->pub_box=nh.advertise<std_msgs::String>("box_switch",2);
        this->pub_ruler=nh.advertise<std_msgs::String>("ruler_switch",2);
        this->flag.data="close";
        this->flag_ruler.data="open";
        this->isFirst=true;
        this->sync_flag=true;
    }

    ~State_Machine()
    {
        std::cout<<"调用析构函数！"<<std::endl;
        delete baxter;
        delete camera;
        delete elbow;
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
    double dist(std::vector<double> a,std::vector<double> b)
    {
        return pow((a[0]-b[0]),2)+pow((a[1]-b[1]),2)+pow((a[2]-b[2]),2);
    }
    std::vector<double> getBoxPos()
    {
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
        middle_point[0]+=0.02;
        middle_point[1]-=0.15;
        middle_point[2]+=0.20;
        std::vector<std::vector<double>> path;
        std::vector<double> tarJnt;
        std::vector<double> lower={-1.7016, -1.347, -3.0541, -0.05, -3.059, -1.5707, -3.059};//抓箱子关节的特殊限位
        std::vector<double> upper={1,  1.2471,  3.0541,  2.618, 3.059,  2.094,   3.059};//抓箱子关节的特殊限位
        std::vector<double> Pos1;
        Pos1=baxter->getJntPos(R_ARM) ;
        for(auto it=Pos1.begin();it!=Pos1.end();it++)
        {
            std::cout<<*it<<" ";
        }
        while(Pos1[0]==0)
        {
            Pos1=baxter->getJntPos(R_ARM);
            ROS_ERROR("G!");
        }
        baxter->cart2Joints(baxter->getJntPos(R_ARM),middle_point,tarJnt,R_ARM,TRAC_IK::Manip1,lower,upper);
        baxter->rightMoveOnce(tarJnt);
        ros::Duration(0.5).sleep();
        baxter->cart2Joints(baxter->getJntPos(R_ARM),first_end,tarJnt,R_ARM,TRAC_IK::Manip1,lower,upper);
        baxter->rightMoveOnce(tarJnt);
        ros::Duration(0.5).sleep();
         //开始补偿
        // path.clear();
        // path.push_back(getCurPos(R_ARM));
        // path.push_back(first_end);
        // baxter->moveTo(path,0.02,0.01,0.10,0.05,R_ARM);
        // ros::Duration(0.2).sleep();
        std::vector<double> compensation;
        compensation=camera->getRightCompensationInfo();
        auto startTime = std::chrono::high_resolution_clock::now();
        std::vector<double> TarJnt;
        double threshould_1=300.00,threshould_2=2000.00,ratio_k=4;
        //1.位置补偿
        //ros::Duration(1.0).sleep();
        while (pow(compensation[1],2)+pow(compensation[2],2)>threshould_1)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            auto endTime=std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::ratio<1, 1> > duration_s(endTime - startTime);
            //path.clear();
            compensation=camera->getRightCompensationInfo();
            //move
            std::vector<double> PosCur_2=getCurPos(R_ARM);
            std::cout<<std::endl;
            double x_move=double(compensation[1])/(1600*ratio_k);
            double y_move=double(compensation[2])/(1600*ratio_k);
            PosCur_2[1]-=x_move;
            PosCur_2[0]-=y_move;
            PosCur_2[2]=first_end[2];
            PosCur_2[3]=first_end[3];
            PosCur_2[4]=first_end[4];
            PosCur_2[5]=first_end[5];
            baxter->cart2Joints(baxter->getJntPos(R_ARM),PosCur_2,TarJnt,R_ARM,TRAC_IK::Manip1,lower);
            baxter->rightMoveOnce(TarJnt);
            ros::Duration(0.5).sleep();
            if(duration_s.count()>10)
            {
                std::cout<<"Compensation timeout!"<<std::endl;
                break;
            }
        } 
        for(int i=0;i<10;i++)
        {
            this->pub_box.publish(this->flag);
            this->pub_ruler.publish(this->flag_ruler);
            ros::Duration(0.1).sleep();
            // endTime=std::chrono::high_resolution_clock::now();
            // auto duration_s(endTime - startTime);
        }
        // this->flag.data="close";
        // this->pub_box.publish(this->flag);
        // this->pub_ruler.publish(this->flag_ruler);
        std::vector<double> final_end=getCurPos(R_ARM);
        std::cout<<"最终箱子坐标"<<std::endl;
        for(auto it=final_end.begin();it!=final_end.end();it++)
        {

            std::cout<<*it<<" "<<std::endl;
        }

        for(int i=0;i<6;i++)
        {
            TarGet.push_back(final_end[i]);
        }
        TarGet[0]-=0.03;//补偿的固定误差
        TarGet[2]=0.04;
        TarGet[3]=-3.14;
        TarGet[4]=0;
        TarGet[5]=-1.57;

        ros::Duration(2.0).sleep();
        // std::vector<double>CurPos=final_end;
        // std::vector<double>CurPos2=final_end;
        // CurPos2[0]+=0.10;
        // CurPos2[1]-=0.05;
        // CurPos2[2]+=0.10;
        // path.clear();
        // path.push_back(CurPos);
        // path.push_back(CurPos2);
        // baxter->moveTo(path,0.02,0.01,0.10,0.05,R_ARM);
        return final_end;
    }
    void GraspRuler(std::vector<double> objPos,std::vector<double> TarPos,std::string armID)
    {
        ROS_INFO("Start Grasping!");
        std::vector<double> objPos_temp,objPos_temp2;
        std::vector<double> tarPos_temp;
        std::vector<std::vector<double>> path;
        path.clear();
        path.push_back(getCurPos(armID));
        objPos_temp.resize(6);
        objPos_temp.clear();
        tarPos_temp.resize(6);
        tarPos_temp.clear();
        for(int i=0;i<6;i++)
        {
            objPos_temp.push_back(objPos.at(i));
            objPos_temp[3]=3.14;
            objPos_temp[4]=0;
            objPos_temp[5]=3.14;
            objPos_temp2.push_back(objPos.at(i));
        }
        objPos_temp.at(2)+=upDistance1;
        //ROS_INFO("3");
        for(int i=0;i<6;i++)
        {
            tarPos_temp.push_back(TarPos.at(i));
        }
        tarPos_temp[2]+=upDistance2;
        tarPos_temp[3]=-3.14;
        tarPos_temp[4]=0;
        tarPos_temp[5]=-1.57;
        //ROS_INFO("2");
        path.push_back(objPos_temp);
        baxter->moveTo(path,0.02,0.01,0.15,0.07,armID);//移到目标块正上方
        path.clear();
        ros::Duration(0.5).sleep();
        //开始补偿
        path.push_back(getCurPos(armID));
        path.push_back(objPos_temp);
        baxter->moveTo(path,0.02,0.01,0.10,0.07,armID);
        path.clear();
        std::vector<double> compensation;
        if(this->color=="blue")
        {
            compensation=camera->getBlueCompensationInfo();
        }
        else
        {
            compensation=camera->getRedCompensationInfo();
        }
        double threshould_1=50.00,threshould_2=2000.00,ratio_k=3;
        //1.位置补偿
        ros::Duration(1.0).sleep();
        auto startTime = std::chrono::high_resolution_clock::now();
        while (pow(compensation[1],2)+pow(compensation[2],2)>threshould_1)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            auto endTime=std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::ratio<1, 1> > duration_s(endTime - startTime);
            path.clear();
            if(this->color=="blue")
            {
                compensation=camera->getBlueCompensationInfo();
            }
            else
            {
                compensation=camera->getRedCompensationInfo();
            }
            //move
            std::vector<double> PosCur_2=getCurPos(armID);
            path.push_back(PosCur_2);
            std::cout<<std::endl;
            double x_move=double(compensation[1])/(1600*ratio_k);
            double y_move=double(compensation[2])/(1600*ratio_k);
            PosCur_2[1]-=x_move;
            PosCur_2[0]-=y_move;
            PosCur_2[2]=0.312;
            // PosCur_2[3]=3.14;
            // PosCur_2[4]=0;
            // PosCur_2[5]=3.14;
            path.push_back(PosCur_2);
            baxter->moveTo(path,0.02,0.01,0.1,0.05,armID);
            std::cout<<std::endl;
            //delay
            ros::Duration(0.7).sleep();
            if(duration_s.count()>20)
            {
                std::cout<<"Compensation timeout!"<<std::endl;
                break;
            }
        }

        //2.角度补偿
        // double total_angle=0;
        // std::map<double,int> mymap;//建立一个字典，key为宽高比，value为转角
        // std::vector<double> CurJnt=baxter->getJntPos(armID);
        // double CurJnt_init=CurJnt[6];
        // double ratio=0;
        // for(int i=0;i<6;i++)
        // {
        //     if(i!=0)
        //     {
        //         CurJnt[6]-=(30*DEGREE2RADIUS);
        //     }
        //     baxter->leftMoveOnce(CurJnt);
        //     ros::Duration(1.5).sleep();
        //     if(this->color=="blue")
        //     {
        //         compensation=camera->getBlueCompensationInfo();
        //     }
        //     else
        //     {
        //         compensation=camera->getRedCompensationInfo();
        //     }
        //     ratio=compensation[0];
        //     std::cout<<"角度为"<<30*i<<"时宽高比为："<<ratio<<std::endl;
        //     mymap.insert(std::pair<double,int>(ratio,30*i));//map基于红黑树，内部有序
        // }
        // ros::Duration(1.0).sleep();
        // auto it=mymap.begin();//首元素，即为宽高比最小的元素
        // double first=it->second;
        // CurJnt[6]=CurJnt_init-it->second*DEGREE2RADIUS;//完成粗校准
        // baxter->leftMoveOnce(CurJnt);
        // ros::Duration(1.0).sleep();
        // mymap.clear();
        // CurJnt=baxter->getJntPos(armID);
        // CurJnt[6]+=15*DEGREE2RADIUS;//在30度范围内细致搜索 -15~15,先顺转15度，然后依次逆转5度
        // double CurJnt_init2=CurJnt[6];
        // for(int i=0;i<6;i++)
        // {
        //     if(i!=0)
        //     {
        //         CurJnt[6]-=(5*DEGREE2RADIUS);
        //     }
        //     baxter->leftMoveOnce(CurJnt);
        //     ros::Duration(1.5).sleep();
        //     if(this->color=="blue")
        //     {
        //         compensation=camera->getBlueCompensationInfo();
        //     }
        //     else
        //     {
        //         compensation=camera->getRedCompensationInfo();
        //     }
        //     ratio=compensation[0];
        //     std::cout<<"角度为"<<first-15+5*i<<"时宽高比为："<<ratio<<std::endl;
        //     mymap.insert(std::pair<double,int>(ratio,5*i));//map基于红黑树，内部有序
        // }
        // ros::Duration(2.0).sleep();
        // it=mymap.begin();
        // CurJnt[6]=CurJnt_init2-it->second*DEGREE2RADIUS;
        // baxter->leftMoveOnce(CurJnt);
        // ros::Duration(1.0).sleep();
        // mymap.clear();
        // total_angle=first-15+it->second;
        // std::cout<<"最终转角为："<<total_angle<<std::endl;
        double total_angle=camera->getAngle();
        std::vector<double> CurJnt=baxter->getJntPos(L_ARM);
        CurJnt[6]-=total_angle*DEGREE2RADIUS;
        baxter->leftMoveOnce(CurJnt);
        ros::Duration(1.0).sleep();
        //将两只手移动到魔尺两端
        std::vector<double> CenterPos=getCurPos(L_ARM);//魔尺中心点坐标
        double ruler_angle=total_angle-90;//魔尺与y方向夹角
        std::vector<double> LeftPos=CenterPos;//左手目标位置
        std::vector<double> RightPos=CenterPos;//右手目标位置
        double ruler=0.135;//设置左，右手到魔尺中心的距离为12cm
        LeftPos[0]-=ruler*sin(ruler_angle*DEGREE2RADIUS);
        LeftPos[1]+=ruler*cos(ruler_angle*DEGREE2RADIUS);
        RightPos[0]+=ruler*sin(ruler_angle*DEGREE2RADIUS);
        RightPos[1]-=ruler*cos(ruler_angle*DEGREE2RADIUS);
        path.clear();
        path.push_back(getCurPos(L_ARM));
        path.push_back(LeftPos);
        baxter->moveTo(path,0.02,0.01,0.10,0.05,L_ARM);//左手到魔尺上方
        ros::Duration(1.0).sleep();
        std::vector<double> Right_Middle=RightPos;
        Right_Middle[2]=0.42;
        path.clear();
        path.push_back(getCurPos(R_ARM));
        path.push_back(Right_Middle);
        path.push_back(RightPos);
        baxter->moveTo(path,0.02,0.01,0.10,0.05,R_ARM);
        ros::Duration(1.0).sleep();
        // path.clear();
        // path.push_back(getCurPos(R_ARM));
        // path.push_back(RightPos);
        // baxter->moveTo(path,0.02,0.01,0.07,0.05,R_ARM);//再动一遍，消除误差
        // ros::Duration(1.0).sleep();

        //双线程动作开始
        auto thd1 = std::async(std::launch::async, &State_Machine::left_pick,this);//左右手将魔尺夹起来
        auto thd2 = std::async(std::launch::async, &State_Machine::right_pick, this);
        thd1.wait();
        thd2.wait();


        std::vector<double> Left=getCurPos(L_ARM);
        std::vector<double> Right=getCurPos(R_ARM);
        std::vector<double> CenterPos2;
        CenterPos2.clear();
        for(int i=0;i<6;i++)
        {
            CenterPos2.push_back((Left[i]+Right[i])/2.0);
        }
        CenterPos2[5]=3.14;

        thd1 = std::async(std::launch::async, &State_Machine::left_rotate,this,CenterPos,ruler,ruler_angle);//左右手绕魔尺中心同步旋转至水平
        thd2 = std::async(std::launch::async, &State_Machine::right_rotate, this,CenterPos,ruler,ruler_angle);
        thd1.wait();
        thd2.wait();

        ros::Duration(0.7).sleep();
        double twist_angle=45;//左右手反向转30度
        thd1 = std::async(std::launch::async, &State_Machine::left_twist,this,twist_angle);//左右手将魔尺掰弯
        thd2 = std::async(std::launch::async, &State_Machine::right_twist, this,twist_angle);
        thd1.wait();
        thd2.wait();
        ros::Duration(0.7).sleep();
        //LeftandRightMove();
        // std::cout<<"同步断点："<<std::endl;
        // int a;
        // std::cin>>a;
        std::vector<double> check1=getCurPos(L_ARM);
        LeftandRightMove2();
        std::vector<double> check2=getCurPos(L_ARM);
        if(dist(check1,check2)<0.01)
        {
            ROS_INFO("SyncMove Failed!");
            //throw "exception";
            LeftandRightMove();
        }
        //ros::Duration(1.0).sleep();

        // ros::Duration(1.0).sleep();
        // std::vector<double> LeftEnd=TarGet;
        // LeftEnd[1]+=0.135;
        // LeftEnd[2]+=upDistance2;
        // std::vector<double> targetJnt;
        // baxter->cart2Joints(baxter->getJntPos(L_ARM),LeftEnd,targetJnt,L_ARM);
        // baxter->leftMoveOnce(targetJnt);
        // ros::Duration(1.0).sleep();
        
        thd1 = std::async(std::launch::async, &State_Machine::left_place,this);//左右手将魔尺放到箱子中并离开
        thd2 = std::async(std::launch::async, &State_Machine::right_place, this);
        thd1.wait();
        thd2.wait();
        ROS_INFO("Successfully put one ruler!");
        baxter->reSetArms(8);
    }
    void left_pick()//左手夹魔尺
    {
        std::vector<std::vector<double>> path;
        std::vector<double> CurPos=getCurPos(L_ARM);
        std::vector<double> EndPos=CurPos;
        EndPos[2]=CurPos[2]-upDistance1-0.01;//系统误差的小补偿
        path.push_back(CurPos);
        path.push_back(EndPos);
        baxter->moveTo(path,0.02,0.01,0.10,0.05,L_ARM);
        ros::Duration(0.7).sleep();
        baxter->grip(L_ARM);//夹持
        ros::Duration(0.7).sleep();
        path.clear();
        path.push_back(getCurPos(L_ARM));
        path.push_back(CurPos);//抬升
        baxter->moveTo(path,0.02,0.01,0.10,0.05,L_ARM);
        //ros::Duration(1.0).sleep();
    }

    void right_pick()//右手夹魔尺
    {
        std::vector<std::vector<double>> path;
        std::vector<double> CurPos=getCurPos(R_ARM);
        std::vector<double> EndPos=CurPos;
        EndPos[2]=CurPos[2]-upDistance1;
        path.push_back(CurPos);
        path.push_back(EndPos);
        baxter->moveTo(path,0.02,0.01,0.10,0.05,R_ARM);
        ros::Duration(0.7).sleep();
        baxter->grip(R_ARM);//夹持
        ros::Duration(0.7).sleep();
        path.clear();
        path.push_back(getCurPos(R_ARM));
        path.push_back(CurPos);//抬升
        baxter->moveTo(path,0.02,0.01,0.10,0.05,R_ARM);
        //ros::Duration(1.0).sleep();
    }

    void left_rotate(std::vector<double> CenterPos,double radius,double angle)//左手绕魔尺中心旋转
    {
        //思路：圆弧插补
        //std::vector<std::vector<double>> path;
        std::vector<double> TarPos=CenterPos;
        double new_angle=angle;
        std::vector<double> TarJnts;
        TarJnts.clear();
        for(int i=1;i<=3;i++)
        {
            TarPos=CenterPos;
            new_angle=angle-angle*i/3.0;
            TarPos[0]-=radius*sin(new_angle*DEGREE2RADIUS);
            TarPos[1]+=radius*cos(new_angle*DEGREE2RADIUS);
            baxter->cart2Joints(baxter->getJntPos(L_ARM),TarPos,TarJnts,L_ARM);
            TarJnts[6]+=angle/3.0*i*DEGREE2RADIUS;
            baxter->leftMoveOnce(TarJnts);
            ros::Duration(0.7).sleep();
            TarJnts.clear();
        }

    }

    void right_rotate(std::vector<double> CenterPos,double radius,double angle)//右手绕魔尺中心旋转
    {
        std::vector<double> TarPos=CenterPos;
        double new_angle=angle;
        std::vector<double> TarJnts;
        TarJnts.clear();
        for(int i=1;i<=3;i++)
        {
            TarPos=CenterPos;
            new_angle=angle-angle*i/3.0;
            TarPos[0]+=radius*sin(new_angle*DEGREE2RADIUS);
            TarPos[1]-=radius*cos(new_angle*DEGREE2RADIUS);
            baxter->cart2Joints(baxter->getJntPos(R_ARM),TarPos,TarJnts,R_ARM);
            TarJnts[6]+=angle/3.0*i*DEGREE2RADIUS;
            baxter->rightMoveOnce(TarJnts);
            ros::Duration(0.7).sleep();
            TarJnts.clear();
        }
        ROS_INFO("Circle Compensation Finished!");
    }

    void left_twist(double angle)//左手掰弯
    {
        std::vector<double> JntPos=baxter->getJntPos(L_ARM);
        JntPos[6]+=angle*DEGREE2RADIUS;//左手顺时针转
        baxter->leftMoveOnce(JntPos);
        ros::Duration(0.7).sleep();
    }

    void right_twist(double angle)//右手掰弯
    {
        std::vector<double> JntPos=baxter->getJntPos(R_ARM);
        JntPos[6]-=angle*DEGREE2RADIUS;//右手逆时针转
        baxter->rightMoveOnce(JntPos);
        ros::Duration(0.7).sleep();
    }
    std::vector<std::vector<double>> path_planning(std::vector<double> input,std::vector<double> output)
    {
        //保证左，右手路径规划方式相同，相对位置不变
        std::vector<std::vector<double>> path;
        path.push_back(input);
        //1.z方向抬高
        std::vector<double> MidPoint_1=input;
        MidPoint_1[2]=output[2]-0.05;
        path.push_back(MidPoint_1);
        //2.x,y方向平移
        std::vector<double> MidPoint_2,MidPoint_3,MidPoint_4;
        MidPoint_2=MidPoint_1;
        // MidPoint_3=MidPoint_1;
        // MidPoint_4=MidPoint_1;
        MidPoint_2[0]=(output[0]-input[0])/2.0+input[0];//插入三等分点，保证走直线
        MidPoint_2[1]=(output[1]-input[1])/2.0+input[1];
        path.push_back(MidPoint_2);
        // MidPoint_3[0]=2*(output[0]-input[0])/3.0+input[0];
        // MidPoint_3[1]=2*(output[1]-input[1])/3.0+input[1];
        // path.push_back(MidPoint_3);
        // MidPoint_4[0]=output[0];
        // MidPoint_4[1]=output[1];
        // path.push_back(MidPoint_4);
        //3.z方向下降
        path.push_back(output);
        return path;
    }
    void LeftandRightMove2()//新尝试的同步器
    {
        std::vector<double> Left_target=TarGet;//左手目标位置
        Left_target[1]+=0.135;
        Left_target[2]+=upDistance2;
        std::vector<double> CurPos1=getCurPos(L_ARM);
        Left_target[3]=CurPos1[3];
        Left_target[4]=CurPos1[4];
        Left_target[5]=CurPos1[5];
        std::vector<std::vector<double>> point_left=path_planning(CurPos1,Left_target);//保证左右手中间的路径点是同一种规划得到的
        ROS_INFO("Point_left:");
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<6;j++)
            {
                std::cout<<point_left[i][j]<<" ";
            }
            std::cout<<std::endl;
        }
        std::vector<double> CurPos2=getCurPos(R_ARM);
        ros::Duration(0.3).sleep();
        baxter->AsyncmoveTo(point_left,CurPos1,CurPos2,0.02,0.01,0.10,0.05,L_ARM);
        //ros::Duration(1.0).sleep();
        // std::vector<double> comp_right=getCurPos(R_ARM);
        // std::vector<std::vector<double>> path;
        // path.clear();
        // path.push_back(getCurPos(L_ARM));
        // std::vector<double> comp_left;
        // for(int i=0;i<6;i++)
        // {
        //     comp_left.push_back(comp_right[i]+CurPos1[i]-CurPos2[i]);
        // }
        // path.push_back(comp_left);
        // baxter->moveTo(path,0.02,0.01,0.07,0.04,L_ARM);
    }
    void LeftandRightMove()//由于moveTo无法保证实时同步，只能分开移动
    {
        std::vector<double> Left_target=TarGet;//左手目标位置
        Left_target[1]+=0.135;
        Left_target[2]+=upDistance2;
        std::vector<double> CurPos1=getCurPos(L_ARM);
        Left_target[3]=CurPos1[3];
        Left_target[4]=CurPos1[4];
        Left_target[5]=CurPos1[5];
        std::vector<std::vector<double>> point_left=path_planning(CurPos1,Left_target);//保证左右手中间的路径点是同一种规划得到的
        std::vector<double> Right_target=TarGet;//右手目标位置
        Right_target[1]-=0.135;
        Right_target[2]+=upDistance2;
        std::vector<double> CurPos2=getCurPos(R_ARM);
        Right_target[3]=CurPos2[3];
        Right_target[4]=CurPos2[4];
        Right_target[5]=CurPos2[5];
        std::vector<std::vector<double>> point_right=path_planning(CurPos2,Right_target);
        ros::Duration(1.0).sleep();
        std::vector<std::vector<double>> path_left;
        std::vector<std::vector<double>> path_right;
        
        std::ofstream ofs(FILE_PATH,std::ios_base::app);
        std::ofstream ofs2(FILE_PATH2,std::ios_base::app);
        std::ofstream ofs3(FILE_PATH3,std::ios_base::app);
        std::ofstream ofs4(FILE_PATH4,std::ios_base::app);

        for(int i=0;i<5;i++)
        {
            if(i<4)
            {
                path_left.clear();
                path_left.push_back(point_left[i]);
                path_left.push_back(point_left[i+1]);
                path_right.clear();
                path_right.push_back(point_right[i]);
                path_right.push_back(point_right[i+1]);
            }
            auto thd1 = std::async(std::launch::async, &BaxterController::moveTo,baxter,path_left,0.02,0.01,0.07,0.04,L_ARM);//左右手将魔尺移到箱子上方
            auto thd2 = std::async(std::launch::async, &BaxterController::moveTo,baxter,path_right,0.02,0.01,0.07,0.04,R_ARM);//左右手将魔尺移到箱子上方
            std::vector<double> left_Pos=getCurPos(L_ARM);
            std::vector<double> right_Pos=getCurPos(R_ARM);
            for(auto it=left_Pos.begin();it!=left_Pos.end();it++)
            {
                ofs<<*it<<",";
            }
            ofs<<std::endl;
            for(auto it=right_Pos.begin();it!=right_Pos.end();it++)
            {
                ofs2<<*it<<",";
            }
            ofs2<<std::endl;
            for(auto it=point_left[i].begin();it!=point_left[i].end();it++)
            {
                ofs3<<*it<<",";
            }
            ofs3<<std::endl;
            for(auto it=point_right[i].begin();it!=point_right[i].end();it++)
            {
                ofs4<<*it<<",";
            }
            ofs4<<std::endl;
            thd1.wait();
            thd2.wait();
            ros::Duration(0.3).sleep();
        }
    }

    void left_place()//左手放置并离开
    {
        std::vector<std::vector<double>> path;
        std::vector<double> CurPos=getCurPos(L_ARM);
        std::vector<double> EndPos=CurPos;
        std::vector<double> FinalPos=CurPos;
        FinalPos[2]+=0.05;
        EndPos[2]-=0.24;//往下放
        path.push_back(CurPos);
        path.push_back(EndPos);
        baxter->moveTo(path,0.02,0.01,0.07,0.05,L_ARM);
        ros::Duration(0.5).sleep();
        baxter->release(L_ARM);//松开
        ros::Duration(0.5).sleep();
        path.clear();
        path.push_back(EndPos);
        path.push_back(FinalPos);
        baxter->moveTo(path,0.02,0.01,0.15,0.07,L_ARM);//抬起来
        ros::Duration(0.5).sleep();   
    }

    void right_place()//右手放置并离开
    {
        std::vector<std::vector<double>> path;
        std::vector<double> CurPos=getCurPos(R_ARM);
        std::vector<double> EndPos=CurPos;
        std::vector<double> FinalPos=CurPos;
        FinalPos[2]+=0.05;
        EndPos[2]-=0.24;//往下放
        path.push_back(CurPos);
        path.push_back(EndPos);
        baxter->moveTo(path,0.02,0.01,0.07,0.05,R_ARM);
        ros::Duration(0.5).sleep();
        baxter->release(R_ARM);//松开
        ros::Duration(0.5).sleep();
        path.clear();
        path.push_back(EndPos);
        path.push_back(FinalPos);
        baxter->moveTo(path,0.02,0.01,0.15,0.07,R_ARM);//抬起来
        ros::Duration(0.5).sleep(); 
    }


    void state(std::vector<std::vector<double>> ValidRuler)
    {
        std::vector<double> objPos;
        objPos.clear();
        objPos=ValidRuler[0];
        GraspRuler(objPos,TarGet,L_ARM);
    }
    std::vector<std::vector<double>> GetRuler()//获得真正有效的块的信息
    {
        std::vector<std::vector<double>> bluePos=camera->getBlueRulerInfo();
        std::vector<std::vector<double>> redPos=camera->getRedRulerInfo();
        std::vector<std::vector<double>> objPos;
        GETINFORMATION:
        std::cout<<"Input color"<<std::endl;
        std::cin>>this->color;

        if (this->color=="red")
        {
            int redsize=redPos.size();
            if(redsize==1)
            {
                objPos=redPos;
            }
            else
            {
                if(redPos[0][0]>redPos[1][0])
                {
                    objPos.push_back(redPos[0]);
                }
                else
                {
                    objPos.push_back(redPos[1]);
                }
            }
        }
        else if(this->color=="blue")
        {
            int bluesize=bluePos.size();
            if(bluesize==1)
            {
                objPos=bluePos;
            }
            else
            {
                if(bluePos[0][0]>bluePos[1][0])
                {
                    objPos.push_back(bluePos[0]);
                }
                else
                {
                    objPos.push_back(bluePos[1]);
                }
            }
        }
        else
        {
            goto GETINFORMATION;
        }
        // objPos.clear();
        // int bluesize=bluePos.size();
        // int redsize=redPos.size();
        // if(bluesize==1&&redsize==2)
        // {
        //     objPos=bluePos;//两红一蓝选蓝
        //     this->color="blue";
        // }
        // else if(bluesize==2&&redsize==1)
        // {
        //     objPos=redPos;//两蓝一红选红
        //     this->color="red";
        // }
        // else if(bluesize>0&&redsize>0)
        // {
        //     objPos==redPos;//无法判断时选红，鸿运当头。。。
        //     this->color="red";
        // }
        // else
        // {
        //     objPos=objPos;//如果有一种颜色为空，返回空
        // }
        return objPos;
    }
    void process()
    {
        std::vector<double> dcw=getBoxPos();
        ros::Duration(1.0).sleep();
        std::vector<std::vector<double>> Ruler;
        Ruler.clear();
        Ruler=GetRuler();
        int size;
        size=Ruler.size();
        ros::spinOnce();
        while(size!=0)
        {
            state(Ruler);
            ros::Duration(1.0).sleep();
            Ruler.clear();
            Ruler=GetRuler();
            size=Ruler.size();
            ros::spinOnce();
        }
        baxter->reSetArms(8);
        ros::Duration(2.0).sleep();
        ROS_INFO("Task Three finished!");
    }
    
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"task_three");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();
    State_Machine task3(nh);
    try
    {
        task3.process();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::cout<<"主程序检测到异常！";
        task3.~State_Machine();
        std::cout<<"对象已经析构！"<<std::endl;
    }
    spinner.stop();
    return 0;
}