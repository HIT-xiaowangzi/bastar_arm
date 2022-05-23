//
// Created by hit on 2020/11/28.
//

#ifndef WRC_HIT_LIB_FORTH_TASK_H
#define WRC_HIT_LIB_FORTH_TASK_H

#include <ros/ros.h>
#include "baxtercontroller.h"
#include "camera_info.h"

namespace FORTH_TASK
{
    class ForthTask
    {
    public:
        ForthTask(ros::NodeHandle nh3);
        ~ForthTask();

        bool setup();
        void start();
        void stop();
        bool imitation(const std::vector<double> &obj_pos, const std::string &armID);
        void taskSwitch(bool enable);
        void initTask();
        void controlThreadFunc();

    private:
        ros::NodeHandle nh3;
        //比赛所有线程控制变量
        volatile bool _forthTaskLoop;
        //比赛任务执行开关变量
        volatile bool _forthTaskControlStart;
        BaxterController* baxter;
        BaxterController* baxterRightArm;
        CameraInfo* camera;
    };
}

#endif //WRC_HIT_LIB_FORTH_TASK_H
