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
#include <std_msgs/String.h>
#define L_ARM "left"
#define R_ARM "right"

const double PI=3.1415926535897;
const double upDistance1=0.27;//上方的一个余量
const double upDistance2=0.40;//箱子高度
const double x_bias=0.10;
const double y_bias=0.10;
const float DEGREE2RADIUS = 0.0174533;
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
    bool isFirstMsg;
    std_msgs::String flag;
    std_msgs::String flag_cattle;
    ros::Publisher pub_box;
    ros::Publisher pub_cattle;
    int count;//第0次抓取
public:
    State_Machine(ros::NodeHandle nh)
    {
        this->nh=nh;
        baxter=new BaxterController(nh);
        camera=new RealsenseInfo2(nh);
        elbow=new ElbowController(nh);
        init_center.resize(3);
        init_center.clear();
        isFirstMsg=true;
        TarGet.resize(3);
        TarGet.clear();
        this->pub_box=nh.advertise<std_msgs::String>("box_switch",2);
        this->pub_cattle=nh.advertise<std_msgs::String>("cattle_switch",2);
        this->flag.data="close";
        this->flag_cattle.data="open";
        this->count=0;
        baxter->reSetArms(8);
    }

    ~State_Machine()
    {
        delete baxter;
        delete camera;
        delete elbow;
    }

    //把给学长实验的代码用上
    double** Rotx(double alpha)
    {
        double** ans=new double*[3];
        for(int i=0;i<3;i++)
        {
            ans[i]=new double[3];
        }
        ans[0][0]=1;ans[0][1]=0;ans[0][2]=0;
        ans[1][0]=0;ans[1][1]=cos(alpha);ans[1][2]=-sin(alpha);
        ans[2][0]=0;ans[2][1]=sin(alpha);ans[2][2]=cos(alpha);
        return ans;
    }

    double** Roty(double beta)
    {
        double** ans=new double*[3];
        for(int i=0;i<3;i++)
        {
            ans[i]=new double[3];
        }
        ans[0][0]=cos(beta);ans[0][1]=0;ans[0][2]=sin(beta);
        ans[1][0]=0;ans[1][1]=1;ans[1][2]=0;
        ans[2][0]=-sin(beta);ans[2][1]=0;ans[2][2]=cos(beta);
        return ans;
    }
    double** Rotz(double gamma)
    {
        double** ans=new double*[3];
        for(int i=0;i<3;i++)
        {
            ans[i]=new double[3];
        }
        ans[0][0]=cos(gamma);ans[0][1]=-sin(gamma);ans[0][2]=0;
        ans[1][0]=sin(gamma);ans[1][1]=cos(gamma);ans[1][2]=0;
        ans[2][0]=0;ans[2][1]=0;ans[2][2]=1;
        return ans;
    }
    double** MulMatrix(double** Matrix1,double** Matrix2)
    {
        double** Matrix=new double*[3];
        int i=0,j=0;
        for(i=0;i<3;i++)
        {
            Matrix[i]=new double [3];
        }
        for(i=0;i<3;i++)
        {
            for(j=0;j<3;j++)
            {
                Matrix[i][j]=0;
            }
        }
        for(i=0;i<3;i++)
        {
            for(j=0;j<3;j++)
            {
                for(int k=0;k<3;k++)
                {
                    Matrix[i][j]+=Matrix1[i][k]*Matrix2[k][j];
                }
            }
        }
        return Matrix;
    }
    double** GetRotMatrix(double alpha,double beta,double gamma)
    {
        double** Rx=Rotx(alpha);
        double** Ry=Roty(beta);
        double** Rz=Rotz(gamma);
        double** result=MulMatrix(MulMatrix(Rz,Ry),Rx);
        return result;
    }
    //
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
        baxter->cart2Joints(baxter->getJntPos(R_ARM),middle_point,tarJnt,R_ARM,TRAC_IK::Manip1,lower,upper);
        baxter->rightMoveOnce(tarJnt);
        ros::Duration(0.5).sleep();
        baxter->cart2Joints(baxter->getJntPos(R_ARM),first_end,tarJnt,R_ARM,TRAC_IK::Manip1,lower,upper);
        baxter->rightMoveOnce(tarJnt);
        ros::Duration(0.5).sleep();
        std::vector<double> compensation;
        compensation=camera->getRightCompensationInfo();
        auto startTime = std::chrono::high_resolution_clock::now();
        std::vector<double> TarJnt;
        double threshould_1=300.00,threshould_2=2000.00,ratio_k=4;
        //1.位置补偿
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
            ros::Duration(0.7).sleep();
            if(duration_s.count()>10)
            {
                std::cout<<"Compensation timeout!"<<std::endl;
                break;
            }
        } 
        for(int i=0;i<10;i++)
        {
            this->pub_box.publish(this->flag);
            this->pub_cattle.publish(this->flag_cattle);
            ros::Duration(0.1).sleep();
            // endTime=std::chrono::high_resolution_clock::now();
            // auto duration_s(endTime - startTime);
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
        final_end[0]-=0.03;
        TarGet[0][0]-=0.03;
        TarGet[0][1]=TarGet[0][1]-0.08;//左偏
        TarGet[0][2]=0.04;
        TarGet[0][3]=-3.14;
        TarGet[0][5]=-1.57;
        TarGet[1][0]-=0.03;
        TarGet[1][1]=TarGet[1][1]+0.06;//右偏
        TarGet[1][2]=0.04;
        TarGet[1][3]=-3.14;
        TarGet[1][5]=-1.57;
        TarGet[2][0]-=0.03;
        TarGet[2][2]=0.18;
        TarGet[2][3]=-3.14;
        TarGet[2][5]=-1.57;


        ros::Duration(2.0).sleep();
        std::vector<double>CurPos=final_end;
        std::vector<double>CurPos2=final_end;
        CurPos2[0]+=0.05;
        CurPos2[1]-=0.05;
        CurPos2[2]+=0.15;
        path.clear();
        path.push_back(CurPos);
        path.push_back(CurPos2);
        baxter->moveTo(path,0.02,0.01,0.10,0.05,R_ARM);
        return final_end;
    }

    double dist(std::vector<double> a,std::vector<double> b)
    {
        return pow((a[0]-b[0]),2)+pow((a[1]-b[1]),2)+pow((a[2]-b[2]),2);
    }

    
    
    std::vector<std::vector<double>> GetValidBlocks()//获得真正有效的块的信息
    {
        std::vector<std::vector<double>> objPos=camera->getBlockInfo();
        std::vector<std::vector<double>> ValidBlocks;
        ValidBlocks.clear();
        int size=objPos.size();
        for(int i=0;i<size;i++)
        {
            ValidBlocks.push_back(objPos[i]);
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
            objPos_temp2.push_back(objPos.at(i));
        }
        objPos_temp.at(2)+=upDistance1;
        for(int i=0;i<6;i++)
        {
            tarPos_temp.push_back(tarPos.at(i));
        }
        tarPos_temp[2]+=upDistance2;
        tarPos_temp[3]=-3.14;
        tarPos_temp[4]=0;
        tarPos_temp[5]=-1.57;
        path.push_back(objPos_temp);
        baxter->moveTo(path,0.02,0.01,0.15,0.07,armID);//移到目标块正上方
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
        double threshould_1=50.00,threshould_2=2000.00,ratio_k=2;
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
            baxter->moveTo(path,0.02,0.01,0.1,0.05,armID);
            std::cout<<std::endl;
            //delay
            ros::Duration(0.7).sleep();
            if(duration_s.count()>25)
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
        baxter->moveTo(path,0.02,0.01,0.15,0.07,armID);//抓块动作要慢
        ros::Duration(1.0).sleep();
        //抖动
        double shake_dist=0.02;
        for(int i=0;i<1;i++)
        {
            CurPos=getCurPos(L_ARM);
            double** RotMatrix=GetRotMatrix(CurPos[3],CurPos[4],CurPos[5]);//3-alpha-x,4-beta-y,5-
            std::vector<double> shake_point1=CurPos;
            std::vector<double> shake_point2=CurPos;
            shake_point1[0]+=shake_dist*RotMatrix[1][0];
            shake_point1[1]+=shake_dist*RotMatrix[1][1];
            shake_point1[2]+=shake_dist*RotMatrix[1][2];
            shake_point2[0]-=shake_dist*RotMatrix[1][0];
            shake_point2[1]-=shake_dist*RotMatrix[1][1];
            shake_point2[2]-=shake_dist*RotMatrix[1][2];
            std::vector<double> tarJnt;
            tarJnt.clear();
            baxter->cart2Joints(baxter->getJntPos(L_ARM),shake_point1,tarJnt,L_ARM);
            baxter->leftMoveOnce(tarJnt);
            ros::Duration(0.05).sleep();
            baxter->cart2Joints(baxter->getJntPos(L_ARM),shake_point2,tarJnt,L_ARM);
            baxter->leftMoveOnce(tarJnt);
            ros::Duration(0.1).sleep();
            baxter->cart2Joints(baxter->getJntPos(L_ARM),CurPos,tarJnt,L_ARM);
            baxter->leftMoveOnce(tarJnt);
            
        }
        baxter->grip(armID);
        ros::Duration(0.5).sleep();
        std::vector<double> EndPos=getCurPos(L_ARM);
        path.clear();
        path.push_back(EndPos);
        EndPos[2]+=upDistance1;
        EndPos[3]=3.14;
        EndPos[4]=0;
        EndPos[5]=3.14;
        path.push_back(EndPos);
        baxter->moveTo(path,0.02,0.01,0.15,0.07,armID);
        ros::Duration(0.5).sleep();
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
        CurPos2[1]+=0.125;//向y方向移12.5cm，防止把牛带倒
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
        // middle_point.resize(6);
        // middle_point.clear();
        // for(int i=0;i<6;i++)
        // {
        //     middle_point.push_back(TarGet[0][i]);
        // }
        // //middle_point[1]-=0.03;
        // middle_point[2]+=upDistance2;
        path.push_back(start_point);
        //path.push_back(middle_point);
        path.push_back(tarPos_temp);
        baxter->moveTo(path,0.02,0.01,0.07,0.05,OtherarmID);//移到目标位置正上方
        path.clear();
        ros::Duration(0.5).sleep();
        path.push_back(getCurPos(OtherarmID));
        path.push_back(tarPos);
        baxter->moveTo(path,0.02,0.01,0.1,0.07,OtherarmID);
        baxter->release(OtherarmID);
        path.clear();
        ros::Duration(0.5).sleep();
        path.push_back(tarPos);
        tarPos_temp[2]+=0.15;
        path.push_back(tarPos_temp);
        baxter->moveTo(path,0.02,0.01,0.25,0.1,OtherarmID);
        ROS_INFO("Successfully place one cattle!");
        //isPicking=false;
    }

    
    void state(int size,std::vector<std::vector<double>> ValidBlocks)//状态识别的太烂了，目标点改一下
    {
        int index=3-size;
        std::vector<double> tarPos;
        tarPos.clear();
        if(this->count<3)
        {
            tarPos=TarGet[this->count];
            this->count++;
        }
        else
        {
            tarPos=TarGet[2];
        }
        // if(first_flag==true)
        // {
        //     for(int i=0;i<6;i++)
        //     {
        //         tarPos.push_back(TarGet[0][i]);
        //     }
        //     first_flag=false;
        // }
        // else
        // {
        //     for(int i=0;i<6;i++)
        //     {
        //         tarPos.push_back(TarGet[index][i]);
        //     }
        // }
        std::vector<double> objPos;
        objPos.clear();
        objPos=ValidBlocks[0];
        GraspCattle(objPos,tarPos,L_ARM);
    }

    void process()
    {
        std::vector<double> dcw=getBoxPos();
        // ros::Duration(2.0).sleep();
        // init_center=Camera_init();
        ros::Duration(1.0).sleep();
        std::vector<std::vector<double>> ValidBlocks;
        ValidBlocks.clear();
        ValidBlocks=GetValidBlocks();
        int size;
        size=ValidBlocks.size();
        ROS_INFO("size:[%d]",size);
        ros::spinOnce();
        //baxter->reSetArms(8);
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
        //ros::Duration(2.0).sleep();
        box_process(dcw);
        ROS_INFO("Task Two finished!");
    }
    void box_process(std::vector<double> dcw)
    {
        //std::vector<double> dcw=getBoxPos();
        std::vector<double> left_elbow_pos=dcw;
        std::vector<double> right_elbow_pos=dcw;
        baxter->reSetArms(8);
        ros::Duration(2.0).sleep();
        left_elbow_pos[1]+=0.33;
        left_elbow_pos[2]=0.035;
        left_elbow_pos[3]=-1.57;
        left_elbow_pos[4]=0;
        left_elbow_pos[5]=0;
        right_elbow_pos[1]-=0.37;
        right_elbow_pos[2]=0.035;
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
        std::vector<double> leftelbow=elbowCurPos(L_ARM);
        std::vector<double> rightelbow=elbowCurPos(R_ARM);
        std::vector<double> endPos=leftelbow;
        endPos[2]+=0.08;
        std::vector<std::vector<double>> path;
        path.clear();
        path.push_back(leftelbow);
        path.push_back(endPos);
        elbow->AsyncmoveTo(path,leftelbow,rightelbow,0.02,0.01,0.07,0.04,L_ARM);
    }
    void left_move(std::vector<double> init_left)
    {
        std::vector<double>G=init_left;
        std::vector<std::vector<double>> path;
        path.clear();
        path.push_back(elbowCurPos(L_ARM));
        path.push_back(init_left);
        elbow->moveTo(path,0.02,0.01,0.10,0.05,L_ARM);
        //elbow->moveCart(init_left,L_ARM);
    }
    void right_move(std::vector<double> init_right)
    {
        std::vector<double>G=init_right;
        std::vector<std::vector<double>> path;
        path.clear();
        path.push_back(elbowCurPos(R_ARM));
        path.push_back(init_right);
        elbow->moveTo(path,0.02,0.01,0.10,0.05,R_ARM);
        //elbow->moveCart(init_right,R_ARM);
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
        elbow->moveTo(path,0.02,0.01,0.1,0.05,L_ARM);
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
        elbow->moveTo(path,0.02,0.01,0.1,0.05,R_ARM);
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
        // path.clear();
        // path.push_back(G2);
        // path.push_back(G3);
        // path.push_back(G4);
        // elbow->moveTo(path,0.02,0.01,0.07,0.05,L_ARM);
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
        // path.clear();
        // path.push_back(G2);
        // path.push_back(G3);
        // path.push_back(G4);
        // elbow->moveTo(path,0.02,0.01,0.07,0.05,R_ARM);

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
                //std::cout<<"LeftTor:"<<LeftTor[1]<<std::endl;
                //std::cout<<"bias:"<<bias<<std::endl;
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
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"task_two_armcontrol");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();
    State_Machine task2(nh);
    ros::Duration(2.0).sleep();
    task2.process();
    spinner.stop();
    return 0;
}