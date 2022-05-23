//
// Created by hit on 2020/11/11.
//

#ifndef WRC_HIT_LIB_THIRD_TASK_H
#define WRC_HIT_LIB_THIRD_TASK_H

#include <ros/ros.h>
#include "baxtercontroller.h"
#include "camera_info.h"

namespace THIRD_TASK
{
    class ThirdTask
    {
    public:
        ThirdTask(ros::NodeHandle nh3);
        ~ThirdTask();

        bool setup();
        void start();
        void stop();

        bool graspCup(const std::vector<double> &visual_cup1_pos, const std::string &armID);
        void taskSwitch(bool enable);
        void initTask();
        void controlThreadFunc();
        void CeshiR(const std::string &armID);
        void CeshiL(const std::string &armID);
        void Ceshi3(const std::string &armID);
        void Ceshi4(const std::string &armID);


    private:
        ros::NodeHandle nh3;
        //比赛所有线程控制变量
        volatile bool _thirdTaskLoop;
        //比赛任务执行开关变量
        volatile bool _thirdTaskControlStart;
        BaxterController* baxter;
        // 比赛任务线程
        std::thread _taskControlThread;
        CameraInfo* camera;
    };
}

#endif //WRC_HIT_LIB_THIRD_TASK_H
