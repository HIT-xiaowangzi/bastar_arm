//
// Created by hit on 2020/11/11.
//

#ifndef WRC_HIT_LIB_FIRST_TASK_H
#define WRC_HIT_LIB_FIRST_TASK_H

#include <ros/ros.h>
#include "baxtercontroller.h"
#include "camera_info.h"


namespace FIRST_TASK
{
    class FirstTask
    {
    public:
        FirstTask(ros::NodeHandle nh);
        ~FirstTask();

        bool setup();
        void start();
        void stop();

        void taskSwitch(bool enable);
        void initTask();
        void controlThreadFunc();

        bool graspNormalObj(const std::vector<std::vector<double>> &obj_pos,const double normal_move_y);
        bool graspOverlapObj(const std::vector<std::vector<double>> &obj_pos,const double overlap_move_x, const double overlap_move_y);
        bool graspMobileObj(const std::vector<std::vector<double>> & center_circle, const std::vector<std::vector<double>> &tar_pos);
        bool leftArmPickAndPlace(const std::vector<double> &objPos, const std::vector<double> tarPos, const std::string &armID);
        bool rightArmPickAndPlace(const std::vector<double> &objPos, const std::vector<double> tarPos);

        bool oneArmPickAndPlaceMobile(const std::vector<double> CirclePos, const std::vector<double> &tar_pos,
                                      const std::string &armID);
    private:
        ros::NodeHandle nh;
        //比赛所有线程控制变量
        volatile bool _firstTaskLoop;
        //比赛任务执行开关变量
        volatile bool _firstTaskControlStart;
        BaxterController* baxter;
        BaxterController* baxterRightArm;

        // 比赛任务线程
        std::thread _taskControlThread;
        CameraInfo* camera;
    };
}

#endif //WRC_HIT_LIB_FIRST_TASK_H
