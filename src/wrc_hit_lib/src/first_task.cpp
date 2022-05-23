//
// Created by hit on 2020/11/11.
//

#include <thread>
#include <future>
#include "first_task.h"
#include "camera_info.h"
#include <pthread.h>
namespace FIRST_TASK {
    namespace {
        const float DEGREE2RADIUS = 0.0174533;

        const std::string L_ARM = "left";
        const std::string R_ARM = "right";
    }

    FirstTask::FirstTask(ros::NodeHandle node) {
        this->nh = node;
        _firstTaskLoop = false;
        _firstTaskControlStart = false;
        taskSwitch(true);

        baxter = new BaxterController(nh);
        baxterRightArm = new BaxterController(nh);
        camera = new CameraInfo(nh);
    }

    FirstTask::~FirstTask() {
        taskSwitch(false);
        delete baxter;
        delete baxterRightArm;
        delete camera;
    }

    bool FirstTask::setup() {
        initTask();
        return true;
    }

    void FirstTask::initTask() {
        taskSwitch(true);
        _firstTaskControlStart = true;
    }

    void FirstTask::stop() {
        taskSwitch(false);
        std::cout << std::right << "\033[1;31m[STOP the first task]\033[0m" << std::endl;
    }

    void FirstTask::taskSwitch(bool enable) {
        _firstTaskLoop = enable;
    }

    void FirstTask::start() {
        while (_firstTaskLoop) {
            if (!_firstTaskControlStart) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                std::cout << std::right << "\033[1;31m[Please start the first task]\033[0m" << std::endl;
                continue;
            }
            std::cout << std::right << "\033[1;31m[Start the first task]\033[0m" << std::endl;
            auto startTime = std::chrono::high_resolution_clock::now();
            std::cout << " The time is ： " << std::chrono::system_clock::to_time_t(startTime) << std::endl;

            baxter->reSetArms(1); // 双臂摆放在胸前
            std::vector<std::vector<double>> visual_obj_pos = camera->getCameraInfo();                   //获取视觉信息,五个普通的物体
            std::vector<std::vector<double>> visual_overlap_obj_pos = camera->getCameraOverlapInfo();    //获取视觉信息,两个重叠的物体

            for (int i = 0; i < visual_obj_pos.size(); ++i)
                visual_obj_pos[i][2] = 0.02;         // 普通物体z方向高度重置
            visual_overlap_obj_pos[0][2] = 0.02;     // 重叠物体的z方向高度重置, z方向较低的物体
            visual_overlap_obj_pos[1][2] = 0.03;     // 重叠物体中z方向较高的物体
            const double height_circle = 0.038;      // 圆盘初始高度重置
            const double mobile_move_y = 0.20;       // 移动物体向桌边移动的距离
            const double normal_move_y = 0.30;       // 普通物体向桌边移动的距离
            const double overlap_move_x = 0.20;      // 重叠物体为了避免终点相同放置冲突, 需要在x方向移动的变量
            const double overlap_move_y = 0.20;      // 重叠物体为了避免终点相同放置冲突, 需要在y方向移动的变量

            std::cout << std::right << "\033[1;32m[Taking picture by Kinect V2 ]\033[0m" << std::endl;
            std::vector<std::vector<double>> normal_obj_pos;
            std::vector<std::vector<double>> overlap_obj_pos;  // 重叠物体初始位置
            std::vector<std::vector<double>> overlap_obj_tar;  // 重叠物体最终位置
            std::vector<std::vector<double>> mobile_obj_pos;
            std::vector<std::vector<double>> mobile_obj_tar;
            std::vector<std::vector<double>> normal_help;      // 中间变量, 辅助物体排序
            std::vector<std::vector<double>> center_of_circle; // 圆盘中心的位置
            normal_obj_pos.clear();
            normal_obj_pos.clear();
            mobile_obj_pos.clear();
            overlap_obj_pos.clear();
            normal_help.clear(); //
            // 对五个普通的物体位置沿着y方向从大到小进行排序,依靠y坐标
            while (!visual_obj_pos.empty()) {
                int k = visual_obj_pos.size() - 1, j;
                double CurValue_double = visual_obj_pos[k][1];   // y坐标
                std::vector<double> CurValue_vector = {visual_obj_pos[k][0], visual_obj_pos[k][1], visual_obj_pos[k][2],
                                                       visual_obj_pos[k][3], visual_obj_pos[k][4]};
                visual_obj_pos.pop_back();

                if (normal_help.size() == 0)
                    j = 0;
                else
                    j = normal_help.size() - 1;

                while (!normal_help.empty() && CurValue_double > normal_help[j][1]) {
                    visual_obj_pos.push_back(
                            {normal_help[normal_help.size() - 1][0], normal_help[normal_help.size() - 1][1],
                             normal_help[normal_help.size() - 1][2], normal_help[normal_help.size() - 1][3],
                             normal_help[normal_help.size() - 1][4]});
                    normal_help.pop_back();
                }
                normal_help.push_back(CurValue_vector);
            }

            center_of_circle = camera->getCenterCircleInfo();    //获取视觉信息,圆盘中心的位置

            if (center_of_circle[0][1] >= 0)  // 移动物体终点位置
                mobile_obj_tar.push_back({center_of_circle[0][0], center_of_circle[0][1] + mobile_move_y, // 移动物体的终点位置
                                          center_of_circle[0][2] - height_circle});   // 如果圆盘中心y方向大于0, 终点坐标应为左边
            else
                mobile_obj_tar.push_back({center_of_circle[0][0], center_of_circle[0][1] - mobile_move_y, // 移动物体的终点位置
                                          center_of_circle[0][2] - height_circle});   // 如果圆盘中心y方向小于0, 终点坐标应为右边
            center_of_circle[0][2] = height_circle;  // 圆盘中心高度重置
            /////////////////运行代码/////////////////////
            if (!graspNormalObj(normal_help, normal_move_y))          // 抓取正常摆放的5个物体
                return;
            if (!graspOverlapObj(visual_overlap_obj_pos, overlap_move_x, overlap_move_y))    // 抓取重叠的2个物体
                return;
            if (!graspMobileObj(center_of_circle, mobile_obj_tar))     // 抓取移动的物体,视觉会给传递信息用哪个手抓取
                return;
            ////////////////////////////////////////////
            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::ratio<1, 1> > duration_s(endTime - startTime);  // 计时器
            std::cout << "The computation time of the first task is： " << duration_s.count() << "  sec " << std::endl;
            break;
        }
    }

    // 抓取正常摆放的5个物体
    bool FirstTask::graspNormalObj(const std::vector<std::vector<double>> &obj_pos, const double normal_move_y) {

        while (true) {
            std::vector<std::pair<int, std::vector<double>>> objCenterPos, tarCenterPos;   //物体初始形状,位置,角度信息; 物体的放置位置,裁判设定
            objCenterPos.clear();
            tarCenterPos.clear();
            std::vector<std::vector<double>> objCenterPosLeft;
            std::vector<std::vector<double>> objCenterPosLeft_help;   // 由于视觉原因,需要对x方向重新由大到小排序
            std::vector<std::vector<double>> tarCenterPosLeft;
            std::vector<std::vector<double>> objCenterPosRight;
            std::vector<std::vector<double>> objCenterPosRight_help;  // 由于视觉原因,需要对x方向重新由大到小排序
            std::vector<std::vector<double>> tarCenterPosRight;
            for (int i = 0; i < obj_pos.size(); ++i) {  // 左右物体分开
                if (obj_pos.at(i).at(1) >= 0)
                    objCenterPosLeft.push_back({obj_pos.at(i).at(0), obj_pos.at(i).at(1), obj_pos.at(i).at(2),
                                                obj_pos.at(i).at(3) * DEGREE2RADIUS, obj_pos.at(i).at(4)});  // 物体目标位置的pushback默认为先左后右
                else
                    objCenterPosRight.push_back({obj_pos.at(i).at(0), obj_pos.at(i).at(1), obj_pos.at(i).at(2),
                                                 obj_pos.at(i).at(3) * DEGREE2RADIUS, obj_pos.at(i).at(4)});

            }

            while (!objCenterPosLeft.empty()) {    // 左侧物体按照x方向从大到小排序
                int k = objCenterPosLeft.size() - 1, j;
                double CurValue_double = objCenterPosLeft[k][0];   // x坐标
                std::vector<double> CurValue_vector = {objCenterPosLeft[k][0], objCenterPosLeft[k][1], objCenterPosLeft[k][2],
                                                       objCenterPosLeft[k][3], objCenterPosLeft[k][4]};
                objCenterPosLeft.pop_back();

                if (objCenterPosLeft_help.size() == 0)
                    j = 0;
                else
                    j = objCenterPosLeft_help.size() - 1;

                while (!objCenterPosLeft_help.empty() && CurValue_double > objCenterPosLeft_help[j][0]) { // x方向
                    objCenterPosLeft.push_back(
                            {objCenterPosLeft_help[objCenterPosLeft_help.size() - 1][0], objCenterPosLeft_help[objCenterPosLeft_help.size() - 1][1],
                             objCenterPosLeft_help[objCenterPosLeft_help.size() - 1][2], objCenterPosLeft_help[objCenterPosLeft_help.size() - 1][3],
                             objCenterPosLeft_help[objCenterPosLeft_help.size() - 1][4]});
                    objCenterPosLeft_help.pop_back();
                }
                objCenterPosLeft_help.push_back(CurValue_vector);
            }

            while (!objCenterPosRight.empty()) {    // 右侧物体按照x方向从大到小排序
                int k = objCenterPosRight.size() - 1, j;
                double CurValue_double = objCenterPosRight[k][0];   // x坐标
                std::vector<double> CurValue_vector = {objCenterPosRight[k][0], objCenterPosRight[k][1], objCenterPosRight[k][2],
                                                       objCenterPosRight[k][3], objCenterPosRight[k][4]};
                objCenterPosRight.pop_back();
                if (objCenterPosRight_help.size() == 0)
                    j = 0;
                else
                    j = objCenterPosRight_help.size() - 1;
                while (!objCenterPosRight_help.empty() && CurValue_double > objCenterPosRight_help[j][0]) { // x方向
                    objCenterPosRight.push_back(
                            {objCenterPosRight_help[objCenterPosRight_help.size() - 1][0], objCenterPosRight_help[objCenterPosRight_help.size() - 1][1],
                             objCenterPosRight_help[objCenterPosRight_help.size() - 1][2], objCenterPosRight_help[objCenterPosRight_help.size() - 1][3],
                             objCenterPosRight_help[objCenterPosRight_help.size() - 1][4]});
                    objCenterPosRight_help.pop_back();
                }
                objCenterPosRight_help.push_back(CurValue_vector);
            }
            for (int i = 0; i < objCenterPosLeft_help.size(); ++i) {
                tarCenterPosLeft.push_back({objCenterPosLeft_help.at(i).at(0), objCenterPosLeft_help.at(i).at(1) + normal_move_y,
                                            objCenterPosLeft_help.at(i).at(2)});  // 最终给定的左边物体的终点坐标
            }

            for (int i = 0; i < objCenterPosRight_help.size(); ++i) {
                tarCenterPosRight.push_back({objCenterPosRight_help.at(i).at(0), objCenterPosRight_help.at(i).at(1) - normal_move_y,
                                            objCenterPosRight_help.at(i).at(2)});  // 最终给定的右边物体的终点坐标
            }

            int temp_max = std::max(objCenterPosLeft_help.size(), objCenterPosRight_help.size());
            int temp_min = std::min(objCenterPosLeft_help.size(), objCenterPosRight_help.size());

            for (int i = 0; i < temp_min; ++i)
            {    //双线程抓取物体
                std::cout << "双线程抓取第 " << i+1 << " 组物体" << std::endl;
                auto thd1 = std::async(std::launch::async, &FirstTask::leftArmPickAndPlace, this, objCenterPosLeft_help[i],
                                       tarCenterPosLeft[i], L_ARM);
                auto thd2 = std::async(std::launch::async, &FirstTask::rightArmPickAndPlace, this, objCenterPosRight_help[i],
                                       tarCenterPosRight[i]);
                thd1.wait();
                thd2.wait();
                baxterRightArm->reSetArms(1);
            }
            if (temp_min == objCenterPosLeft_help.size()) { // 对剩余一侧的物体进行判断,判断用左臂抓取还是右臂
                // 若左侧为最小值,即右侧vector有剩余,应处理右侧vector剩余参数
                for (int i = temp_min; i < temp_max; ++i) {
                    std::cout << "单线程抓取右侧剩余的第 " << i << " 个物体" << std::endl;
                    auto thd1 = std::async(std::launch::async, &FirstTask::rightArmPickAndPlace, this,
                                           objCenterPosRight_help[i], tarCenterPosRight[i]);
                    thd1.wait();
                    baxterRightArm->reSetArms(1);
                }
            } else {
                // 若右侧为最小值,即左侧vector有剩余,应处理左侧vector剩余参数
                for (int i = temp_min; i < temp_max; ++i) {
                    std::cout << "单线程抓取左侧剩余的第 " << i << " 个物体" << std::endl;
                    auto thd1 = std::async(std::launch::async, &FirstTask::leftArmPickAndPlace, this,
                                           objCenterPosLeft_help[i], tarCenterPosLeft[i], L_ARM);
                    thd1.wait();
                    baxter->reSetArms(1);
                }
            }
            return true;
        }
    }

    bool FirstTask::graspOverlapObj(const std::vector<std::vector<double>> &obj_pos, const double overlap_move_x, const double overlap_move_y) {
        while (true)
        {
            std::vector<std::vector<double>> objCenterPos, tarCenterPos;   //物体初始形状,位置,角度信息; 物体的放置位置,裁判设定
            objCenterPos.clear();
            tarCenterPos.clear();
            // z方向较高的物体先进栈,1是高的物体(先抓),0是低的物体(后抓)
            objCenterPos.push_back({obj_pos.at(1).at(0), obj_pos.at(1).at(1), obj_pos.at(1).at(2),
                                    obj_pos.at(1).at(3) * DEGREE2RADIUS, obj_pos.at(1).at(4)});
            objCenterPos.push_back({obj_pos.at(0).at(0), obj_pos.at(0).at(1), obj_pos.at(0).at(2),
                                    obj_pos.at(0).at(3) * DEGREE2RADIUS, obj_pos.at(0).at(4)});
            // 匹配物体顺序,确定终点位置
            if (obj_pos.at(1).at(1) >= 0)
            {
                tarCenterPos.push_back({obj_pos.at(1).at(0) - overlap_move_x, obj_pos.at(1).at(1) + overlap_move_y, obj_pos.at(1).at(2)});
                tarCenterPos.push_back({obj_pos.at(0).at(0) + overlap_move_x, obj_pos.at(0).at(1) + overlap_move_y, obj_pos.at(0).at(2)});
            }
            else
            {
                tarCenterPos.push_back({obj_pos.at(1).at(0) - overlap_move_x, obj_pos.at(1).at(1) - overlap_move_y, obj_pos.at(1).at(2)});
                tarCenterPos.push_back({obj_pos.at(0).at(0) + overlap_move_x, obj_pos.at(0).at(1) - overlap_move_y, obj_pos.at(0).at(2)});
            }
            // 判断用左臂还是右臂抓取,z方向较高的物体先被抓取
            if (objCenterPos[0][1] >= 0) {
                std::cout << " <左臂> 抓取重叠物体 " << std::endl;
                auto thd = std::async(std::launch::async, &FirstTask::leftArmPickAndPlace, this, objCenterPos[0], tarCenterPos[0], L_ARM);
                thd.wait();
                thd = std::async(std::launch::async, &FirstTask::leftArmPickAndPlace, this, objCenterPos[1], tarCenterPos[1], L_ARM);
                thd.wait();
            } else {
                std::cout << " <右臂> 抓取重叠物体 " << std::endl;
                auto thd = std::async(std::launch::async, &FirstTask::rightArmPickAndPlace, this, objCenterPos[0], tarCenterPos[0]);
                thd.wait();
                thd = std::async(std::launch::async, &FirstTask::rightArmPickAndPlace, this, objCenterPos[1], tarCenterPos[1]);
                thd.wait();
            }
            return true;
        }
    }

    // 抓取移动的物体
    bool FirstTask::graspMobileObj(const std::vector<std::vector<double>> &center_circle, const std::vector<std::vector<double>> &tar_pos) {
        while (true)
        {
            std::vector<std::vector<double>> CircleCenterPos, tarCenterPos;   //圆盘中心位置; 物体的放置位置,裁判设定
            CircleCenterPos.clear();
            tarCenterPos.clear();
            CircleCenterPos.push_back({center_circle.at(0).at(0), center_circle.at(0).at(1), center_circle.at(0).at(2), center_circle.at(0).at(3)}); // 圆盘中心位置和物体标号堆栈
            tarCenterPos.push_back({tar_pos.at(0).at(0), tar_pos.at(0).at(1), tar_pos.at(0).at(2)});   // 移动物体终点位置堆栈
            baxter->reSetArms(0);
            if (CircleCenterPos[0][1] >= 0)
                oneArmPickAndPlaceMobile(CircleCenterPos[0], tarCenterPos[0], L_ARM); // 圆盘中心位置, 目标位置, 手臂编号
            else
                oneArmPickAndPlaceMobile(CircleCenterPos[0], tarCenterPos[0], R_ARM);
            return true;
        }
    }

    // 抓取移动的物体
    bool FirstTask::oneArmPickAndPlaceMobile(const std::vector<double> CirclePos, const std::vector<double> &tar_pos,
                                             const std::string &armID) {
        // 采用垂直姿态 进行抓取
        double upDistance;
        if (CirclePos[3] == 0)
            upDistance = 0.11;  // 竖直物体为0
        else
            upDistance = 0.08;  // 水平物体为1
        const double xDistance = 0.03;  // x方向余量,方便手部摄像头把转盘拍全
        std::vector<double> currJnt = baxter->getJntPos(armID);
        std::vector<double> currPos;
        baxter->joint2Cart(currJnt, currPos, armID);   // 正解求末端位置和姿态
        std::vector<double> posHold_0 = {CirclePos[0] + xDistance, CirclePos[1], CirclePos[2] + upDistance, currPos[3], currPos[4],
                                         currPos[5]};  // 后三位为欧拉角
        std::vector<double> jntHold_0;
        baxter->cart2Joints(currJnt, posHold_0, jntHold_0, armID);  // 逆解求得 jntHold_0
        std::cout << "手臂移动到圆盘中心点！！" << std::endl;
        if (armID == L_ARM)
            baxter->leftMoveOnce(jntHold_0);    // 移动至头部摄像头给定的初始位置上方
        else
            baxterRightArm->rightMoveOnce(jntHold_0);    // 移动至头部摄像头给定的初始位置上方
        std::vector<double> compensationInfo;
        for (int i = 0; i < 2; ++i)
        {    // 2次位置补偿
            std::this_thread::sleep_for(std::chrono::seconds(1));   // 暂停,手部摄像头拍照,位置补偿
            if (armID == L_ARM) // 获取移动物体初始信息
                compensationInfo = camera->getLeftCompensationInfo();
            else
                compensationInfo = camera->getRightCompensationInfo();
            if (CirclePos[4] == 0)
            {
                compensationInfo[1] *= 0.0005;         // 需根据高度调整
                compensationInfo[2] *= 0.0005;
            }
            else
            {
                compensationInfo[1] *= 0.00055;         // 需根据高度调整
                compensationInfo[2] *= 0.00055;
            }
            currJnt = baxter->getJntPos(armID);    // 当前各个关节的角度
            baxter->joint2Cart(currJnt, currPos, armID); // 反解得到currPos
            posHold_0 = {currPos[0] + compensationInfo[1], currPos[1] + compensationInfo[2], currPos[2],
                         currPos[3], currPos[4], currPos[5]}; // Euler angle, 补偿
            baxter->cart2Joints(currJnt, posHold_0, jntHold_0, armID);   // 反解
            if (armID == L_ARM)
                baxter->leftMoveOnce(jntHold_0);    // 位置补偿,手部位置移动
            else
                baxter->rightMoveOnce(jntHold_0);
        }
        // 角度补偿
        jntHold_0[6] -= (compensationInfo[0] * DEGREE2RADIUS);  // 末端角度补偿(左臂/右臂)
        if (armID == L_ARM)
            baxter->leftMoveOnce(jntHold_0);               // 补偿角度,手部旋转
        else
            baxter->rightMoveOnce(jntHold_0);
/////   先进行角度补偿
//        std::vector<double> compensationInfo;
//        std::this_thread::sleep_for(std::chrono::seconds(1));
//        if (armID == L_ARM) // 获取移动物体初始信息
//            compensationInfo = camera->getLeftCompensationInfo();
//        else
//            compensationInfo = camera->getRightCompensationInfo();
//        const double jnt_compen = compensationInfo[0] * DEGREE2RADIUS;  // 角度补偿
//        jntHold_0[6] -= jnt_compen;  // 角度补偿
//        if (armID == L_ARM)
//            baxter->leftMoveOnce(jntHold_0);
//        else
//            baxterRightArm->rightMoveOnce(jntHold_0);
/////  再进行位置补偿
//        KDL::Rotation2 rot(jnt_compen);
//        KDL::Vector2 dis_compen;
////        std::this_thread::sleep_for(std::chrono::seconds(1));
////        if (armID == L_ARM) // 获取移动物体初始信息
////            compensationInfo = camera->getLeftCompensationInfo();
////        else
////            compensationInfo = camera->getRightCompensationInfo();
//        if (CirclePos[3] == 0) {
//            dis_compen.x(compensationInfo[1] * 0.0005);
//            dis_compen.y(compensationInfo[2] * 0.0005);
//        } else {
//            dis_compen.x(compensationInfo[1] * 0.00055);
//            dis_compen.y(compensationInfo[2] * 0.00055);
//        }
//        dis_compen = rot * dis_compen;
//        currJnt = baxter->getJntPos(armID);              // 当前各个关节的角度
//        baxter->joint2Cart(currJnt, currPos, armID);     // 正解得到currPos, 末端xyz和欧拉角
//        posHold_0 = {currPos[0] + dis_compen.x(), currPos[1] + dis_compen.y(), currPos[2],
//                     currPos[3], currPos[4], currPos[5]}; // x,y,z, Euler angle
//        if (armID == L_ARM)
//        {
//            baxter->cart2Joints(currJnt, posHold_0, jntHold_0, L_ARM);   // 反解,求得jntHold_0(各个关节的转角)
//            baxter->leftMoveOnce(jntHold_0);
//        }
//        else
//        {
//            baxterRightArm->cart2Joints(currJnt, posHold_0, jntHold_0, R_ARM);   // 反解,求得jntHold_0(各个关节的转角)
//            baxterRightArm->rightMoveOnce(jntHold_0);
//        }

        std::vector<std::vector<double>> paths;
        currJnt = baxter->getJntPos(armID);
        baxter->joint2Cart(currJnt, currPos, armID);
        paths.clear();
        paths.push_back(currPos);
        currPos[2] -= upDistance;
        paths.push_back(currPos);
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));   // 0代表圆柱竖直状态  1代表圆柱平躺状态
        double temp;
        while(1)
        {
            if (armID == L_ARM) // 随时获取移动物体的信息
                compensationInfo = camera->getLeftCompensationInfo();
            else
                compensationInfo = camera->getRightCompensationInfo();
            std::cout << "角度补偿为: " << compensationInfo[0] << std::endl;
            temp = std::sqrt(pow(compensationInfo[1],2) + pow(compensationInfo[2],2));
                if (CirclePos[3] == 0)   // 竖直状态
                {
//                    if(temp <= 17)
                    if (compensationInfo[1] <= 12 && compensationInfo[2] <= 12) // 补偿判断条件
                    {
                        baxter->moveTo(paths, 0.02, 0.01, 0.8, 0.6, armID);  // 向下移动,抓取移动物体
                        baxter->grip(armID);                                 // 抓住
                        break;
                    }
                }
                else
                {
//                if (compensationInfo[1] <= 15 && compensationInfo[2] <= 15)
//                if(temp <= 22 && abs(compensationInfo[0]) <= 5)  // 圆柱平躺状态
                    if(fabs(compensationInfo[0]) <= 5)
                    {
                        baxter->moveTo(paths, 0.02, 0.01, 0.8, 0.6, armID);  // 向下移动,抓取移动物体
                        baxter->grip(armID);                                 // 抓住
                     break;
                    }
                }
        }
        // 抬起手臂->移动到终点位置上方->移动到终点位置
        paths.clear();
        paths.push_back(currPos);
        currPos[2] += upDistance;         // 向上抬起
        paths.push_back(currPos);
        currPos[0] = tar_pos[0];
        currPos[1] = tar_pos[1];
        paths.push_back(currPos);         // 目标位置上方
        currPos[2] = tar_pos[2];
        paths.push_back(currPos);         // 目标位置
        baxter->moveTo(paths, 0.02, 0.01, 0.25, 0.1, armID);    // 3. 抬起手臂->移动到终点位置上方->移动到终点位置
        baxter->release(armID);
        // 抬起手臂
        paths.clear();
        paths.push_back(currPos);
        currPos[2] += upDistance;
        paths.push_back(currPos);
        baxter->moveTo(paths, 0.02, 0.01, 0.25, 0.1, armID);    // 4. 抬起手臂
        baxter->reSetArms(1);
        return true;
    }

    bool FirstTask::leftArmPickAndPlace(const std::vector<double> &objPos, const std::vector<double> tarPos, const std::string &armID) {
        /// 采用垂直姿态 进行抓取
        const double upDistance = 0.09;  //
        std::vector<double> currJnt = baxter->getJntPos(armID);  // 当前各个关节的角度
        std::vector<double> currPos;
        baxter->joint2Cart(currJnt, currPos, armID);  // Fk: obtain currPos 正解
        std::vector<double> posHold_0 = {objPos[0], objPos[1], objPos[2] + upDistance, currPos[3], currPos[4], currPos[5]}; // Euler angle
        std::cout << 'x' << objPos[0] << std::endl;
        std::vector<double> jntHold_0;
        baxter->cart2Joints(currJnt, posHold_0, jntHold_0, armID);  // IK: obtain jntHold_0
        std::cout << "oneArmPickAndPlace->机器人开始运动到Hold位姿！！" << std::endl;
        baxter->leftMoveOnce(jntHold_0);    // 移动至头部摄像头给定的初始位置上方
        std::vector<double> compensationInfo;  // 补偿信息
        std::this_thread::sleep_for(std::chrono::seconds(1));
        compensationInfo = camera->getLeftCompensationInfo();
        const double jnt_compen = compensationInfo[0] * DEGREE2RADIUS;
        jntHold_0[6] -= jnt_compen;  // 角度补偿
        baxter->leftMoveOnce(jntHold_0);

        KDL::Rotation2 rot(jnt_compen);
        KDL::Vector2 dis_compen;

        for (int i = 0; i < 3; ++i)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            compensationInfo = camera->getLeftCompensationInfo();
            if (objPos[4] == 0) {
                dis_compen.x(compensationInfo[1] * 0.0005);
                dis_compen.y(compensationInfo[2] * 0.0005);
            } else {
                dis_compen.x(compensationInfo[1] * 0.00055);
                dis_compen.y(compensationInfo[2] * 0.00055);
            }

            dis_compen = rot * dis_compen;

            currJnt = baxter->getJntPos(L_ARM);              // 当前各个关节的角度
            baxter->joint2Cart(currJnt, currPos, L_ARM);     // 正解得到currPos, 末端xyz和欧拉角
            posHold_0 = {currPos[0] + dis_compen.x(), currPos[1] + dis_compen.y(), currPos[2],
                         currPos[3], currPos[4], currPos[5]}; // x,y,z, Euler angle
            baxter->cart2Joints(currJnt, posHold_0, jntHold_0, L_ARM);   // 反解,求得jntHold_0(各个关节的转角)
            baxter->leftMoveOnce(jntHold_0);
        }

        // 向下移动并抓取
        std::vector<std::vector<double>> paths;
        currJnt = baxter->getJntPos(armID);
        baxter->joint2Cart(currJnt, currPos, armID);
        paths.clear();
        paths.push_back(currPos);
        currPos[2] -= upDistance;   // z
        paths.push_back(currPos);
        baxter->moveTo(paths, 0.02, 0.01, 0.1, 0.07, armID);  // 移动
        baxter->grip(armID);
        // 抬起手臂->移动至目标位置上方->向下移动->释放手爪
        paths.clear();
        paths.push_back(currPos);  // 需要赋值起始位置
        currPos[2] += upDistance;
        paths.push_back(currPos);
        currPos[0] = tarPos[0];
        currPos[1] = tarPos[1];
        paths.push_back(currPos);
        currPos[2] -= upDistance;
        paths.push_back(currPos);
        baxter->moveTo(paths, 0.02, 0.01, 0.25, 0.1, armID);
        baxter->release(armID);
        // 抬起手臂
        paths.clear();
        paths.push_back(currPos);
        currPos[2] += upDistance;
        paths.push_back(currPos);
        baxter->moveTo(paths, 0.02, 0.01, 0.25, 0.1, armID); // path rad eqrad vel acc armID
        return true;
    }

    bool FirstTask::rightArmPickAndPlace(const std::vector<double> &objPos, const std::vector<double> tarPos)
    {
        /// 采用垂直姿态 进行抓取
        const double upDistance = 0.09;  //
        std::vector<double> currJnt = baxter->getJntPos(R_ARM);  // 当前各个关节的角度
        std::vector<double> currPos;
        baxterRightArm->joint2Cart(currJnt, currPos, R_ARM);  // Fk: obtain currPos 正解
        std::vector<double> posHold_0 = {objPos[0], objPos[1], objPos[2] + upDistance, currPos[3], currPos[4], currPos[5]}; // Euler angle
        std::vector<double> jntHold_0;
        baxterRightArm->cart2Joints(currJnt, posHold_0, jntHold_0, R_ARM);  // IK: obtain jntHold_0
        std::cout << "oneArmPickAndPlace->机器人开始运动到Hold位姿！！" << std::endl;
        baxterRightArm->rightMoveOnce(jntHold_0);
        std::vector<double> compensationInfo;  // 补偿信息
        // 补偿信息,包含角度,x,y方向的位置补偿, 若是armID臂,则获取armID臂的补偿信息
        std::this_thread::sleep_for(std::chrono::seconds(1));
        compensationInfo = camera->getRightCompensationInfo();
        const double jnt_compen = compensationInfo[0] * DEGREE2RADIUS;
        jntHold_0[6] -= jnt_compen;  // 角度补偿
        baxterRightArm->rightMoveOnce(jntHold_0);

        KDL::Rotation2 rot(jnt_compen);
        KDL::Vector2 dis_compen;

        for (int i = 0; i < 3; ++i)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            compensationInfo = camera->getRightCompensationInfo();
            std::cout << "right compensationInfo = " << compensationInfo[1] << ", " << compensationInfo[2] << std::endl;

            if (objPos[4] == 0) {
                dis_compen.x(compensationInfo[1] * 0.0005);
                dis_compen.y(compensationInfo[2] * 0.0005);
            } else {
                dis_compen.x(compensationInfo[1] * 0.00055);
                dis_compen.y(compensationInfo[2] * 0.00055);
            }

            dis_compen = rot * dis_compen;

            currJnt = baxterRightArm->getJntPos(R_ARM);              // 当前各个关节的角度
            baxterRightArm->joint2Cart(currJnt, currPos, R_ARM);     // 正解得到currPos, 末端xyz和欧拉角
            posHold_0 = {currPos[0] + dis_compen.x(), currPos[1] + dis_compen.y(), currPos[2],
                         currPos[3], currPos[4], currPos[5]}; // x,y,z, Euler angle
            baxterRightArm->cart2Joints(currJnt, posHold_0, jntHold_0, R_ARM);   // 反解,求得jntHold_0(各个关节的转角)
            baxterRightArm->rightMoveOnce(jntHold_0);
        }

        // 向下移动并抓取
        std::vector<std::vector<double>> paths;
        currJnt = baxterRightArm->getJntPos(R_ARM);
        baxterRightArm->joint2Cart(currJnt, currPos, R_ARM);
        paths.clear();
        paths.push_back(currPos);
        currPos[2] -= upDistance;   // z
        paths.push_back(currPos);
        baxterRightArm->moveTo(paths, 0.02, 0.01, 0.1, 0.07, R_ARM);  // 移动
        baxterRightArm->grip(R_ARM);
        // 抬起手臂->移动至目标位置上方->向下移动->释放手爪
        paths.clear();
        paths.push_back(currPos);  // 需要赋值起始位置
        currPos[2] += upDistance;
        paths.push_back(currPos);
        currPos[0] = tarPos[0];
        currPos[1] = tarPos[1];
        paths.push_back(currPos);
        currPos[2] -= upDistance;
        paths.push_back(currPos);
        baxterRightArm->moveTo(paths, 0.02, 0.01, 0.25, 0.1, R_ARM);
        baxterRightArm->release(R_ARM);
        // 抬起手臂
        paths.clear();
        paths.push_back(currPos);
        currPos[2] += upDistance;
        paths.push_back(currPos);
        baxterRightArm->moveTo(paths, 0.02, 0.01, 0.25, 0.1, R_ARM); // path rad eqrad vel acc armID
        return true;
    }
}