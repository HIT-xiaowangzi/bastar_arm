//
// Created by hit on 2020/11/28.
//

#include "forth_task.h"
#include "camera_info.h"
#include <thread>

namespace FORTH_TASK {
    namespace {
        const float DEGREE2RADIUS = 0.0174533;
        const std::string L_ARM = "left";
        const std::string R_ARM = "right";
    }

    ForthTask::ForthTask(ros::NodeHandle node) {
        this->nh3 = node;
        _forthTaskLoop = false;
        _forthTaskControlStart = false;
        taskSwitch(true);
        baxter = new BaxterController(nh3);
        camera = new CameraInfo(nh3);
    }

    ForthTask::~ForthTask() {
        taskSwitch(false);
        delete baxter;
    }

    bool ForthTask::setup() {
        initTask();
        return true;
    }

    void ForthTask::initTask() {
        taskSwitch(true);
        _forthTaskControlStart = true;
    }

    void ForthTask::stop() {
        taskSwitch(false);
        std::cout << std::right << "\033[1;31m[STOP the forth task]\033[0m" << std::endl;
    }

    void ForthTask::taskSwitch(bool enable) {
        _forthTaskLoop = enable;
    }

    void ForthTask::start() {
        while (_forthTaskLoop) {
            if (!_forthTaskControlStart) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                std::cout << std::right << "\033[1;31m[Please start the forth task]\033[0m" << std::endl;
                continue;
            }
            std::cout << std::right << "\033[1;31m[Start the forth task]\033[0m" << std::endl;
            auto startTime = std::chrono::high_resolution_clock::now();
            std::cout << " The time is ： " << std::chrono::system_clock::to_time_t(startTime) << std::endl;
            baxter->reSetArms(6);
            std::vector<double> forthtask_order_info;
            while (1)
            {
                forthtask_order_info = camera->getForthCompensationInfo();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                if (forthtask_order_info[0] == 8)  //zero start
                    break;
                else
                    continue; // 强迫本次循环结束,开始下一次的循环
            }
            std::cout << " 已经检测到zero手势,检测开始!!" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            while (1)
            {
                forthtask_order_info = camera->getForthCompensationInfo();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                // yeah手势,上下移动,z方向
                if (forthtask_order_info[0] == 1)
                {
                    double z_distance = forthtask_order_info[1] * 0.01;
                    std::vector<double> currJnt = baxter->getJntPos(L_ARM);
                    std::vector<double> currPos;
                    baxter->joint2Cart(currJnt, currPos, L_ARM);   // 正解求末端位置和姿态
                    std::vector<double> posHold_0 = {currPos[0], currPos[1] , currPos[2] + z_distance,
                                                     currPos[3], currPos[4], currPos[5]};  // 后三位为欧拉角
                    std::vector<double> jntHold_0;
                    baxter->cart2Joints(currJnt, posHold_0, jntHold_0, L_ARM);  // 逆解求得 jntHold_0
                    baxter->leftJntCmdPub(jntHold_0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    std::cout << " 已经检测到yeah手势并开始运动!!" << std::endl;
                }
                // ok手势,左右移动,y方向
                else if (forthtask_order_info[0] == 2)
                {
                    double y_distance = forthtask_order_info[2] * 0.01;
                    std::vector<double> currJnt = baxter->getJntPos(L_ARM);
                    std::vector<double> currPos;
                    baxter->joint2Cart(currJnt, currPos, L_ARM);   // 正解求末端位置和姿态
                    std::vector<double> posHold_0 = {currPos[0], currPos[1] + y_distance, currPos[2],
                                                     currPos[3], currPos[4], currPos[5]};  // 后三位为欧拉角
                    std::vector<double> jntHold_0;
                    baxter->cart2Joints(currJnt, posHold_0, jntHold_0, L_ARM);  // 逆解求得 jntHold_0
                    baxter->leftJntCmdPub(jntHold_0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    std::cout << " 已经检测到ok手势并开始运动!!" << std::endl;
                }
                // five手势, 前后移动, x方向
                else if (forthtask_order_info[0] == 3)
                {
                    double x_distance = forthtask_order_info[1] * 0.01;
                    std::vector<double> currJnt = baxter->getJntPos(L_ARM);
                    std::vector<double> currPos;
                    baxter->joint2Cart(currJnt, currPos, L_ARM);   // 正解求末端位置和姿态
                    std::vector<double> posHold_0 = {currPos[0] + x_distance, currPos[1] , currPos[2],
                                                     currPos[3], currPos[4], currPos[5]};  // 后三位为欧拉角
                    std::vector<double> jntHold_0;
                    baxter->cart2Joints(currJnt, posHold_0, jntHold_0, L_ARM);  // 逆解求得 jntHold_0
                    baxter->leftJntCmdPub(jntHold_0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    std::cout << " 已经检测到five手势并开始运动!!" << std::endl;
                }
                // six手势, 末端旋转
                else if (forthtask_order_info[0] == 4)
                {
                    std::vector<double> currJnt = baxter->getJntPos(L_ARM);
                    currJnt[6] += 0.1;
                    baxter->leftJntCmdPub(currJnt);    //
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    std::cout << " 已经检测到six手势并开始运动!!" << std::endl;
                }
                // one手势,grip命令
                else if (forthtask_order_info[0] == 5)
                {
                    baxter->grip(L_ARM);
                    std::cout << " 已经检测到one手势并开始运动!!" << std::endl;
                }
                // three手势, release命令
                else if (forthtask_order_info[0] == 6)
                {
                    baxter->release(L_ARM);
                    std::cout << " 已经检测到three手势并开始运动!!" << std::endl;
                }
                // good end
                else
                    baxter->reSetArms(1);
                    std::cout << " 已经检测到good手势并结束程序!!" << std::endl;
                    break;
            }
            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::ratio<1, 1> > duration_s(endTime - startTime);  // 计时器
            std::cout << " The computation time of the forth task is： " << duration_s.count() << "  sec " << std::endl;
            break;
        }
    }
}
