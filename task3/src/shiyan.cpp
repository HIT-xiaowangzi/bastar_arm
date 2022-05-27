#include<iostream>
#include "ros/ros.h"
#include "baxtercontroller.h"
#include <thread>
#include <future>
#include <math.h>
#include <vector>
#define L_ARM "left"
#define R_ARM "right"
using namespace std;
class Shiyan
{
public:
    ros::NodeHandle nh;
    BaxterController* baxter;
    Shiyan(ros::NodeHandle nh)
    {
        baxter=new BaxterController(nh);
    }
    ~Shiyan()
    {
        delete baxter;
    }
    vector<double> getCurPos(string armID)
    {
        vector<double>currJnt=baxter->getJntPos(armID);
        vector<double> Cart;
        Cart.resize(6);
        Cart.clear();
        baxter->joint2Cart(currJnt,Cart,armID);
        return Cart;
    }
    void test()
    {
        vector<double> endPos={0.8,0.2,0.1,3.14,0,3.14};
        vector<vector<double>> path;
        path.push_back(getCurPos(L_ARM));
        path.push_back(endPos);
        baxter->moveTo(path,0.02,0.02,0.10,0.05,L_ARM);
    }

};
int main(int argc,char** argv)
{
    ros::init(argc,argv,"shiyan");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();
    Shiyan sh(nh);
    sh.test();
    std::cout<<"1";
    spinner.stop();
    return 0;
}