//Created by wangzirui on 2022/2/22
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "baxtercontroller.h"
#include "camera_info.h"
#include <thread>
#include <future>

#define L_ARM "left"
#define R_ARM "right"
const float upDistance=0.20;
class Test
{
private:
    ros::NodeHandle nh;
    BaxterController* baxter;
    CameraInfo* camera;
public:
    Test(ros::NodeHandle nh)
    {
        this->nh=nh;
        baxter=new BaxterController(this->nh);
        camera=new CameraInfo(this->nh);
    }
    ~Test()
    {
        delete this->baxter;
        //delete this->camera;
    }
    void Reset(int choice)
    {
        baxter->reSetArms(choice);
    }
    std::vector<double> getCart(const std::string &armID)
    {
        std::vector<double>currJnt=baxter->getJntPos(armID);
        std::vector<double> Cart;
        baxter->joint2Cart(currJnt,Cart,armID);
        return Cart;
    }
    void PickOneBlock(const std::vector<double> &objPos,const std::vector<double> &tarPos,const std::string &armID)
    {

        std::vector<double>curJnt=baxter->getJntPos(armID);
        std::vector<double> tarJnt;
        baxter->cart2Joints(curJnt,objPos,tarJnt,armID);
        baxter->rightMoveOnce(tarJnt);
        //baxter->release(armID);
        std::vector<double>objPos_temp;
        objPos_temp.resize(6);
        objPos_temp.clear();
        for(int i=0;i<6;i++)
        {
            objPos_temp[i]=objPos[i];
        }
        objPos_temp[2]+=upDistance;
        //std::this_thread::sleep_for(std::chrono::milliseconds(30));
        baxter->moveCart(objPos_temp,armID);
        std::vector<double> tarPos_temp;
        tarPos_temp.resize(6);
        tarPos_temp.clear();
        for(int i=0;i<6;i++)
        {
            tarPos_temp[i]=tarPos[i];
        }
        tarPos_temp[2]+=upDistance;
        baxter->moveCart(tarPos_temp,armID);
        baxter->moveCart(tarPos,armID);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        baxter->grip(armID);
    }
};
int main(int argc,char** argv)
{
    ros::init(argc,argv,"test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();
    Test test(nh);
    //test.Reset(0);
    //std::vector<double> result=test.getCart(R_ARM);
    std::vector<double>objPos,tarPos;
    objPos.resize(6);
    objPos.clear();
    tarPos.resize(6);
    tarPos.clear();
    //for(auto it=result.begin();it!=result.end();it++)
    // {
    //     std::cout<<*it<<" ";
    // }
    // objPos.resize(6);
    // objPos.clear();
    // for(int i=3;i<6;i++)
    // {
    //     objPos[i]=result[i];
    //     tarPos[i]=result[i];
    // }
    objPos[0]=0.6;
    objPos[1]=-0.3;
    objPos[2]=0.0;
    objPos[3]=3.14;
    objPos[4]=0;
    objPos[5]=3.14;
    tarPos[0]=0.8;
    tarPos[1]=-0.5;
    tarPos[2]=0.0;
    tarPos[3]=3.14;
    tarPos[4]=0;
    tarPos[5]=3.14;
    test.PickOneBlock(objPos,tarPos,R_ARM);
    spinner.stop();
    return 0;
}