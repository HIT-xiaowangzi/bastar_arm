//
// Created by hit on 2020/11/19.
//

#include <thread>
#include <future>
#include "third_task.h"
#include "camera_info.h"
#include <opencv2/opencv.hpp>

namespace THIRD_TASK {
    namespace {
        const float DEGREE2RADIUS = 0.0174533;
        const std::string L_ARM = "left";
        const std::string R_ARM = "right";
    }

    ThirdTask::ThirdTask(ros::NodeHandle node) {
        this->nh3 = node;
        _thirdTaskLoop = false;
        _thirdTaskControlStart = false;
        taskSwitch(true);
        baxter = new BaxterController(nh3);
        camera = new CameraInfo(nh3);
    }

    ThirdTask::~ThirdTask() {
        taskSwitch(false);
        delete baxter;
    }

    bool ThirdTask::setup() {
        initTask();
        return true;
    }

    void ThirdTask::initTask() {
        taskSwitch(true);
        _thirdTaskControlStart = true;
    }

    void ThirdTask::stop() {
        taskSwitch(false);
        std::cout << std::right << "\033[1;31m[STOP the third task]\033[0m" << std::endl;
    }

    void ThirdTask::taskSwitch(bool enable) {
        _thirdTaskLoop = enable;
    }

    void ThirdTask::start() {
        while (_thirdTaskLoop) {
            if (!_thirdTaskControlStart) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                std::cout << std::right << "\033[1;31m[Please start the third task]\033[0m" << std::endl;
                continue;
            }
            std::cout << std::right << "\033[1;31m[Start the third task]\033[0m" << std::endl;
            auto startTime = std::chrono::high_resolution_clock::now();
            std::cout << " The time is ： " << std::chrono::system_clock::to_time_t(startTime) << std::endl;
//            std::vector<double> visual_cup1_pos = camera->getWideCupInfo();        //获取视觉信息,宽口杯子的位置
            std::vector<double> visual_cup1_pos = {0.60, 0.2, 0.05};   // 0.60, 0.37, 0.05  // 0.60, -0.15, 0.05

//            if (visual_cup1_pos[1] >= 0)   // 判断用左臂抓还是右臂抓
//                baxter->reSetArms(6);      // 左臂水平位置,略向下倾斜
//            else
//                baxter->reSetArms(5);      // 右臂水平位置,略向下倾斜
            baxter->reSetArms(1);
//            CeshiR(R_ARM);
//            CeshiL(L_ARM);
//            baxter->reSetArms(1);

//            Ceshi4(L_ARM);
//            while (1){
//
//            }
//            visual_cup1_pos[2] = -0.01;  /////////////////// 杯子位置的矫正?///////////////////////
            std::cout << std::right << "\033[1;32m[Taking picture by Kinect V2 ]\033[0m" << std::endl;
            if (visual_cup1_pos[1] >= 0)   // 判断宽口杯子的位置,选择手臂抓取杯子
            {
                if (!graspCup(visual_cup1_pos, L_ARM))     // 抓取宽口杯子(宽口杯子位置,左臂)
                    return;
            } else {
                if (!graspCup(visual_cup1_pos, R_ARM))     // 抓取宽口杯子(宽口杯子位置,右臂)
                    return;
            }
            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::ratio<1, 1> > duration_s(endTime - startTime);  // 计时器
            std::cout << " The computation time of the third task is： " << duration_s.count() << "  sec " << std::endl;
            break;
        }
    }

    bool ThirdTask::graspCup(const std::vector<double> &visual_cup1_pos, const std::string &armID) {

        while(true) {
            const double upDistance = 0.11;
            const double upDistance_2 = 0.07; // 此高度为宽口杯子向窄口杯子倒球所准备
            std::vector<double> currJnt = baxter->getJntPos(L_ARM);  // 角度信息
            std::vector<double> currPos;
            baxter->joint2Cart(currJnt, currPos, L_ARM);  // 正解Fk: obtain currPos
            std::vector<double> posHold_0 = {visual_cup1_pos[0], visual_cup1_pos[1], visual_cup1_pos[2] + upDistance,
                                             currPos[3], currPos[4], currPos[5]}; // Euler angle
            std::vector<double> jntHold_0;
            baxter->cart2Joints(currJnt, posHold_0, jntHold_0, L_ARM);  // 逆解IK: obtain jntHold_0 各个关节的角度
            std::cout << "手臂开始运动到装有小球的杯子上方！！" << std::endl;
            if (armID == L_ARM)
                baxter->leftMoveOnce(jntHold_0);    // 先移动到杯子的上方
            else
                baxter->leftMoveOnce(jntHold_0);
            // 此时需要识别杯子位置,反馈抓取的xyz信息
            int max_loop = 0;
            while (1)  // 循环补偿
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 暂停拍照
                std::vector<double> compensationInfo = camera->getLeftCompensationInfo();   // 腕部相机得到宽口杯子的位置信息
                if (fabs(compensationInfo[0]) <= 5 && fabs(compensationInfo[1]) <= 5)
                    break;
                else
                {
                    compensationInfo[1] *= 0.0005;         // 需根据高度调整
                    compensationInfo[2] *= 0.0005;
                    currJnt = baxter->getJntPos(L_ARM);    // 当前各个关节的角度
                    baxter->joint2Cart(currJnt, currPos, L_ARM); // 正解得到currPos
                    posHold_0 = {currPos[0] + compensationInfo[1], currPos[1] + compensationInfo[2], currPos[2],
                             currPos[3], currPos[4], currPos[5]}; // Euler angle, 补偿
                    baxter->cart2Joints(currJnt, posHold_0, jntHold_0, L_ARM);   // 反解
                    if (armID == L_ARM)
                        baxter->leftMoveOnce(jntHold_0);    // 位置补偿,手部位置移动
                    else
                        baxter->leftMoveOnce(jntHold_0);
                    max_loop++;
                }
                if(max_loop > 20)
                {
                    ROS_INFO("Force to jump the while loop~~~~~");
                    break;
                }   // 防止陷入死循环
            }
            currJnt = baxter->getJntPos(L_ARM);          // 当前各个关节的角度
            baxter->joint2Cart(currJnt, currPos, L_ARM); // 正解得到currPos
            std::vector<double> WideCuptarPos = {currPos[0], currPos[1], currPos[2]};  // 补偿后的宽口杯子目标位置
            double x_incline, y_incline;   // 需要根据末端角度确定
            if (armID == L_ARM)
            {
                x_incline = -0.065;    // 左臂对应关系,需测量
                y_incline = 0.075;
            }
            else
            {
                x_incline = -0.065;    // 右臂对应关系
                y_incline = 0.075;
            }  // 此位置为抓取宽口杯子的位置, 为了末端调整
            WideCuptarPos[0] += x_incline;    // 左臂对应关系
            WideCuptarPos[1] += y_incline;
            std::vector<std::vector<double>> paths;
            // 先将末端提起再变换姿态
            paths.clear();
            paths.push_back(currPos);   //  需要赋值起始位置
            currPos[2] += 0.16;         ///  给定预留高度,方便变换姿态, 末端从垂直变为水平
            paths.push_back(currPos);   //  需要赋值起始位置
            baxter->moveTo(paths, 0.02, 0.01, 0.25, 0.1, armID);
            // 切换位姿
            if (armID == L_ARM)
                baxter->reSetArms(6);
            else
                baxter->reSetArms(6);  // 后续需要更改
            // 以末端倾斜45度的姿态前往上一步记录的xyz位置, 以几近水平位置接近
            currJnt = baxter->getJntPos(L_ARM);
            baxter->joint2Cart(currJnt, currPos, L_ARM);  // 正解Fk: obtain currPos
            //  先高度下降
            paths.clear();
            paths.push_back(currPos);   //  需要赋值起始位置
            currPos[1] += 0.12;         //末端移动到桌子远端
            currPos[2] = -0.08;         /// 该变量可能需要调整
            paths.push_back(currPos);
            baxter->moveTo(paths, 0.02, 0.01, 0.25, 0.1, L_ARM);
            paths.clear();
            paths.push_back(currPos);   //  需要赋值起始位置
            currPos[0] = WideCuptarPos[0];   //  水平接近理想位置
            currPos[1] = WideCuptarPos[1];
            paths.push_back(currPos);
            baxter->moveTo(paths, 0.02, 0.01, 0.25, 0.1, L_ARM);
            baxter->grip(L_ARM);        //  抓取宽口杯子
//            /// 抬高并将右臂移开
//            paths.clear();
//            paths.push_back(currPos);   //  需要赋值起始位置
//            currPos[2] += 0.15;
//            paths.push_back(currPos);
//            baxter->moveTo(paths, 0.02, 0.01, 0.25, 0.1, L_ARM);
//            std::vector<double> temp_left_Jnt = {-0.8981457513069098, -0.5238544390628688, 0.09318933286403888, 1.94662161982659,
//                                            -0.08091748656095557, -1.4258351423394922, -0.060975736318445196};
//            std::vector<double> temp_right_Jnt = {-0.1457281748491143, -0.7742768026851626,0.42721364942608775, 1.3483691125512787,
//                                                  -0.3236699462438223, 1.0895098545956152, -0.45674277959288195};
//            baxter->leftMoveOnce(temp_left_Jnt);
//            baxter->rightMoveOnce(temp_right_Jnt);
            /// 移动到指定位置->开始倒球
            double s_comp_x = 0.040;        // 末端x的补偿      0.117
            double s_comp_y = 0.120;        // y方向倒球的补偿   0.093
            double upDistance_3 = 0.08;     // 该高度为了防止窄口杯子位置改变,宽口杯子移动会打到窄口杯子
            for (int i = 0; i < 200; ++i)
            {
                ///  抬起至窄口杯子的高度, 留有两个高度距离, 一个方便倒球, 一个避免移动过程中打到窄口杯子
                currJnt = baxter->getJntPos(L_ARM);
                baxter->joint2Cart(currJnt, currPos, L_ARM);  // 正解Fk: obtain currPos
                std::vector <double> nar_cup_pos = camera->getNarrowCupInfo();   // 得到窄口杯子的位置信息
                paths.clear();
                paths.push_back(currPos);   //  需要赋值起始位置
                currPos[2] = nar_cup_pos[2] + upDistance_2 + upDistance_3;  // 移动到窄口杯子的高度,留有一定倒球的高度
//                currPos[2] = nar_cup_pos[2] + nar_cup_pos[5] + upDistance_2 + upDistance_3;  // 移动到窄口杯子的高度,留有一定倒球的高度
                paths.push_back(currPos);
                baxter->moveTo(paths, 0.02, 0.01, 0.25, 0.1, armID);
                /// 水平接近窄口杯子
//                int count = 0;
//                while (1)
//                {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                nar_cup_pos = camera->getNarrowCupInfo();
                nar_cup_pos[0] = nar_cup_pos[0] - s_comp_x;     // 此处需要获得话题中的窄口杯子位置补偿信息,待修改(2020.12.02)
//                nar_cup_pos[0] = nar_cup_pos[0] + nar_cup_pos[3];
                nar_cup_pos[1] = nar_cup_pos[1] + s_comp_y;     // 改变窄口杯子终点位置，加上补偿距离，方便倒球
//                nar_cup_pos[0] = nar_cup_pos[1] + nar_cup_pos[4];
                paths.clear();
                paths.push_back(currPos);
                currPos[0] = nar_cup_pos[0];
                currPos[1] = nar_cup_pos[1];     // 接近窄口杯子
                paths.push_back(currPos);
                currPos[2] -= upDistance_3;      // 然后下降到窄口杯子附近
                paths.push_back(currPos);
                baxter->moveTo(paths, 0.002, 0.001, 0.25, 0.1, armID);
//                    if (fabs(currPos[0] - nar_cup_pos[0]) <= 0.005  &&  fabs(currPos[1] - nar_cup_pos[1]) <= 0.005)
//                        break;
//                    else
//                    {
//                        count ++;
//                        if(max_loop > 100)
//                        {
//                            ROS_INFO("I force to jump the while loop~~~~~~~~~");
//                            break;
//                        }   // 防止陷入死循环
//                    }
//                }
                /// 慢慢旋转将球倒出
                std::vector<double> visual_cup2_pos;
                std::vector<double> visual_cup2_pos_temp;
                double temp;
                jntHold_0 = baxter->getJntPos(L_ARM);
                visual_cup2_pos_temp = camera->getNarrowCupInfo();    // 暂存第一次窄口杯子的位置
                /// 直到窄口杯子的位置发生改变,宽口杯子才终止旋转
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                for (int i = 0; i < 2500; ++i)
                {
                    if (i < 500)
                    {
                        jntHold_0[6] += 0.0024434;    // 70
                        if (armID == L_ARM)
                            baxter->leftJntCmdPub(jntHold_0);     // 转动末端倒出小球
                        else
                            baxter->rightJntCmdPub(jntHold_0);
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));   // 倒球需要时间间隔
                            visual_cup2_pos = camera->getNarrowCupInfo();   //获取窄口杯子的位置
                            temp = sqrt(std::pow(visual_cup2_pos_temp[0] - visual_cup2_pos[0], 2) +
                                        std::pow(visual_cup2_pos_temp[1] - visual_cup2_pos[1], 2) +
                                        std::pow(visual_cup2_pos_temp[2] - visual_cup2_pos[2], 2));  // 判断两次杯子的位置差
                            if (temp > 0.03)     // 若前后窄口杯子位置发生改变
                                break;
                            visual_cup2_pos_temp = visual_cup2_pos;  // 保留上一次窄口杯子的位置
                    }
                    else if (500 <= i && i < 1500)      // 70 -> 90, 0.34907
                    {
                        jntHold_0[6] += 0.00034907;
                        if (armID == L_ARM)
                            baxter->leftJntCmdPub(jntHold_0);     // 转动末端倒出小球
                        else
                            baxter->rightJntCmdPub(jntHold_0);
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));   // 倒球需要时间间隔
                        visual_cup2_pos = camera->getNarrowCupInfo();                 //获取窄口杯子的位置
                        temp = sqrt(std::pow(visual_cup2_pos_temp[0] - visual_cup2_pos[0], 2) +
                                    std::pow(visual_cup2_pos_temp[1] - visual_cup2_pos[1], 2) +
                                    std::pow(visual_cup2_pos_temp[2] - visual_cup2_pos[2], 2));  // 判断两次杯子的位置差
                        if (temp > 0.03)     // 前后窄口杯子位置发生改变
                            break;
                        visual_cup2_pos_temp = visual_cup2_pos;
                    }
                    else     //(3000 <= i <= 5000)  // 90 -> 95
                    {
                        jntHold_0[6] += 0.00008727;
                        if (armID == L_ARM)
                            baxter->leftJntCmdPub(jntHold_0);     // 转动末端倒出小球
                        else
                            baxter->rightJntCmdPub(jntHold_0);
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));   // 倒球需要时间间隔
                        visual_cup2_pos = camera->getNarrowCupInfo();   //获取窄口杯子的位置
                        temp = sqrt(std::pow(visual_cup2_pos_temp[0] - visual_cup2_pos[0], 2) +
                                    std::pow(visual_cup2_pos_temp[1] - visual_cup2_pos[1], 2) +
                                    std::pow(visual_cup2_pos_temp[2] - visual_cup2_pos[2], 2));  // 判断两次杯子的位置差
                        if (temp > 0.03)     // 前后窄口杯子位置发生改变
                            break;
                        visual_cup2_pos_temp = visual_cup2_pos;
                    }
                }
                // 末端角度复位
                currJnt = baxter->getJntPos(L_ARM);
                currJnt[6] = -0.060975736318445196;  /// 复位数值可能需要更改
                baxter->leftMoveOnce(currJnt);     // 转动末端倒出小球
                std::this_thread::sleep_for(std::chrono::seconds(2));   // 倒球需要时间间隔
                visual_cup2_pos_temp = {visual_cup2_pos[0], visual_cup2_pos[1], visual_cup2_pos[2]}; // 保存此次循环的窄口杯子位置
            }

//            /// 向下移动并抓取宽口杯子
//            currJnt = baxter->getJntPos(armID);
//            baxter->joint2Cart(currJnt, currPos, armID);  // 正解
//            std::vector<std::vector<double>> paths;
//            paths.clear();
//            paths.push_back(currPos);
//            currPos[2] -= upDistance;   // z
//            paths.push_back(currPos);
//            baxter->moveTo(paths, 0.02, 0.01, 0.1, 0.07, armID);  // 移动
//            baxter->grip(armID);
            /// 移动到窄口杯子上方 updistance_2 的高度和水平具有 horizontal 的距离
//            paths.clear();
//            paths.push_back(currPos);  // 需要赋值起始位置
////            currPos[2] = tar_pos[2] + upDistance;    // 抬起手臂,手臂高度应在窄口杯子的上方
////            paths.push_back(currPos);
////            currPos[0] = tar_pos[0] + horizontal;
////            currPos[1] = tar_pos[1];
////            currPos[2] = tar_pos[2] + upDistance_2;  // 此为预留高度,方便倒球
////            paths.push_back(currPos);
//            currPos[2] += upDistance;
//            paths.push_back(currPos);
//            baxter->moveTo(paths, 0.02, 0.01, 0.1, 0.07, armID);
//            /// 倒球规划
//            currJnt = baxter->getJntPos(armID);
//            for (int i = 0; i < 2500; ++i)
//                {
//                    if (i < 500)
//                    {
//                        currJnt[6] -= 0.0024434;    // 70
//                        if (armID == L_ARM)
//                            baxter->leftJntCmdPub(jntHold_0);     // 转动末端倒出小球
//                        else
//                            baxter->rightJntCmdPub(jntHold_0);
//                        std::this_thread::sleep_for(std::chrono::milliseconds(10));   // 倒球需要时间间隔
//                    }
//                    else if (500 <= i && i < 1500)      // 70 -> 90, 0.34907
//                    {
//                        currJnt[6] -= 0.00034907;
//                        if (armID == L_ARM)
//                            baxter->leftJntCmdPub(jntHold_0);     // 转动末端倒出小球
//                        else
//                            baxter->rightJntCmdPub(jntHold_0);
//                        std::this_thread::sleep_for(std::chrono::milliseconds(10));   // 倒球需要时间间隔
//                    }
//                    else     //(3000 <= i <= 5000)  // 90 -> 95
//                    {
//                        currJnt[6] -= 0.00008727;
//                        if (armID == L_ARM)
//                            baxter->leftJntCmdPub(jntHold_0);     // 转动末端倒出小球
//                        else
//                            baxter->rightJntCmdPub(jntHold_0);
//                        std::this_thread::sleep_for(std::chrono::milliseconds(10));   // 倒球需要时间间隔
//                    }
//                }
            return true;
            }
        ////////结合实际视觉信息的代码////////
//        while (true){          // 左臂倒球,最好在移动在窄口杯子位置的左上方; 右臂倒球,最好移动在窄口杯子位置的右上方
//            const double upDistance = 0.2;
//            const double upDistance_2 = 0.01;    // 此高度为宽口杯子向窄口杯子倒球所准备,末端夹爪与窄口杯子瓶口的垂直距离
//            double horizontal;
//            if (armID == L_ARM)      // 水平预留距离,方便倒球,需根据实际情况调整
//                horizontal = 0.1;      // 左臂倒球,最好在移动在窄口杯子位置的左上方, 10cm为宽口杯子与窄口杯子之间的轴线距离
//            else
//                horizontal = -0.1;     // 右臂倒球,最好移动在窄口杯子位置的右上方
//            std::vector<double> currJnt = baxter->getJntPos(armID);  // 角度信息
//            std::vector<double> currPos;
//            baxter->joint2Cart(currJnt, currPos, armID);  // Fk: obtain currPos
//            std::vector<double> posHold_0 = {cup_pos[0], cup_pos[1], cup_pos[2] + upDistance, currPos[3], currPos[4], currPos[5]}; // Euler angle
//            std::vector<double> jntHold_0;
//            baxter->cart2Joints(currJnt, posHold_0, jntHold_0, armID);  // IK: obtain jntHold_0 各个关节的角度
//            std::cout << "手臂开始运动到装有小球的杯子上方！！" << std::endl;
//            baxter->moveOnce(jntHold_0, armID);     // 先移动到杯子的上方
//            /// 向下移动并抓取宽口杯子
//            currJnt = baxter->getJntPos(armID);
//            baxter->joint2Cart(currJnt, currPos, armID);  // 正解
//            std::vector<std::vector<double>> paths;
//            paths.clear();
//            paths.push_back(currPos);
//            currPos[2] -= upDistance;   // z
//            paths.push_back(currPos);
//            baxter->moveTo(paths, 0.02, 0.01, 0.1, 0.07, armID);  // 移动
//            baxter->grip(armID);
//            /// 移动到窄口杯子上方 updistance_2 的高度
//            paths.clear();
//            paths.push_back(currPos);  // 需要赋值起始位置
//            currPos[2] = cup2_pos[2] + upDistance;    // 抬起手臂,手臂高度应在窄口杯子的上方
//            paths.push_back(currPos);
//            currPos[0] = cup2_pos[0] + horizontal;
//            currPos[1] = cup2_pos[1];
//            currPos[2] = cup2_pos[2] + upDistance_2;  // 此为预留高度,方便倒球
//            paths.push_back(currPos);
//            baxter->moveTo(paths, 0.02, 0.01, 0.1, 0.07, armID);
//            /// 倒球
//            double temp;
//            std::vector<double> visual_cup2_pos_temp;  // 暂存上一次杯子的位置
//            std::vector<double> visual_cup2_pos;       // 存放窄口杯子的位置信息
//            visual_cup2_pos_temp = {cup2_pos[0], cup2_pos[1], cup2_pos[2]};      // 暂存上一次窄口杯子的位置
//            while (1){                                                        // 直到窄口杯子的位置发生改变,宽口杯子才终止旋转
//                std::vector<double> visual_cup2_pos = camera->getCupInfo();   //获取窄口杯子的位置
//                temp = sqrt(std::pow(visual_cup2_pos_temp[0] - visual_cup2_pos[0], 2) + std::pow(visual_cup2_pos_temp[1] - visual_cup2_pos[1], 2)
//                            + std::pow(visual_cup2_pos_temp[2] - visual_cup2_pos[2], 2));  // 判断两次杯子的位置差
//                if (temp > 0.01)        // 前后窄口杯子位置发生改变
//                    break;
//                else
//                {
//                    jntHold_0[6] -= 0.001;    // 每次旋转的角度
//                    baxter->jntCmdPub(jntHold_0, armID);     // 转动末端倒出小球
//                }
//                std::this_thread::sleep_for(std::chrono::milliseconds(10));   // 倒球需要时间间隔
//                visual_cup2_pos_temp = {visual_cup2_pos[0], visual_cup2_pos[1], visual_cup2_pos[2]};
//            }
//            /// 当杯子位置改变时,应当将宽口杯子位置还原?
//            /// 窄口杯子位置发生改变->抬起手臂至新目标位置(窄口杯子)上方->移动至目标位置上方updistance_2的距离 -> 倒球 (过程中窄口杯子的位置会改变两次)
//            for (int i = 0; i < 2; ++i)
//            {   // 窄口杯子位置改变两次
//                visual_cup2_pos = camera->getCupInfo();   //获取窄口杯子的位置
//                currJnt = baxter->getJntPos(armID);
//                baxter->joint2Cart(currJnt, currPos, armID);  // 正解
//                paths.clear();
//                paths.push_back(currPos);  // 需要赋值起始位置
//                currPos[2] = visual_cup2_pos[2] + upDistance;  // 抬起手臂,手臂高度应在窄口杯子的上方
//                paths.push_back(currPos);
//                currPos[0] = visual_cup2_pos[0] + horizontal;
//                currPos[1] = visual_cup2_pos[1];
//                paths.push_back(currPos);
//                currPos[2] = visual_cup2_pos[2] + upDistance_2;  // 此为预留高度,方便倒球
//                paths.push_back(currPos);
//                baxter->moveTo(paths, 0.02, 0.01, 0.1, 0.07, armID);
//                /// 慢慢旋转将球倒出
//                jntHold_0 = baxter->getJntPos(armID);
//                visual_cup2_pos_temp = {visual_cup2_pos[0], visual_cup2_pos[1], visual_cup2_pos[2]}; // 暂存第一次窄口杯子的位置
//                while (1) {                // 直到窄口杯子的位置发生改变,宽口杯子才终止旋转
//                    visual_cup2_pos = camera->getCupInfo();   //获取窄口杯子的位置
//                    temp = sqrt(std::pow(visual_cup2_pos_temp[0] - visual_cup2_pos[0], 2) +
//                                std::pow(visual_cup2_pos_temp[1] - visual_cup2_pos[1], 2)
//                                + std::pow(visual_cup2_pos_temp[2] - visual_cup2_pos[2], 2));  // 判断两次杯子的位置差
//                    if (temp > 0.01)      // 前后窄口杯子位置发生改变
//                        break;
//                    else {
//                        jntHold_0[6] -= 0.001;    // 每次旋转的角度
//                        baxter->jntCmdPub(jntHold_0, armID);     // 转动末端倒出小球
//                    }
//                    std::this_thread::sleep_for(std::chrono::milliseconds(10));   // 倒球需要时间间隔
//                    visual_cup2_pos_temp = {visual_cup2_pos[0], visual_cup2_pos[1], visual_cup2_pos[2]};
//                }
//            }
//            return true;
//        }
    }




    //////////////////////////////////////测试用/////////////////////////////////////
    //////////////////////////////////////测试用/////////////////////////////////////
    //////////////////////////////////////测试用/////////////////////////////////////
    void ThirdTask::CeshiR(const std::string &armID){   //右臂
        std::vector<double> currJnt = baxter->getJntPos(armID);  // 当前各个关节的角度
        std::vector<double> currPos;  // 末端位姿
//        baxter->joint2Cart(currJnt, currPos, armID);  // Fk: obtain currPos 正解
//        std::vector<double> posHold_0 = {0.665, -0.2, 0.30, -1.57, -1.57, 0}; // Euler angle
//        std::vector<double> jntHold_0;
//        baxter->cart2Joints(currJnt, posHold_0, jntHold_0, armID);  // IK: obtain jntHold_0
//        baxter->moveOnce(jntHold_0, armID);    // 水平位置
        std::vector<std::vector<double>> paths;
        currJnt = baxter->getJntPos(armID);  // 当前各个关节的角度
        baxter->joint2Cart(currJnt, currPos, armID);  // Fk: obtain currPos 正解
        paths.clear();
        paths.push_back(currPos);
        currPos[0] -= 0.007;
//        currPos[1] += 0.005;
        currPos[2] += 0.005;
        paths.push_back(currPos);
        baxter->moveTo(paths, 0.02, 0.01, 0.1, 0.07, armID);  // 向下移动,抓取移动物体
//        baxter->jntCmdPub(currPos,armID);
    }

    void ThirdTask::CeshiL(const std::string &armID){
        std::vector<double> currJnt = baxter->getJntPos(armID);  // 当前各个关节的角度
        std::vector<double> currPos;  // 末端位姿
//        baxter->joint2Cart(currJnt, currPos, armID);  // Fk: obtain currPos 正解
//        std::vector<double> posHold_0 = {0.665, 0.2, 0.30, 0, 1.57, -1.57}; // Euler angle
//        std::vector<double> jntHold_0;
//        baxter->cart2Joints(currJnt, posHold_0, jntHold_0, armID);  // IK: obtain jntHold_0
//        baxter->moveOnce(jntHold_0, armID);    // 移动至头部摄像头给定的初始位置上方
        std::vector<std::vector<double>> paths;
        currJnt = baxter->getJntPos(armID);  // 当前各个关节的角度
        baxter->joint2Cart(currJnt, currPos, armID);  // Fk: obtain currPos 正解
        paths.clear();
        paths.push_back(currPos);
        currPos[1] += 0.02;
        paths.push_back(currPos);
        baxter->moveTo(paths, 0.02, 0.01, 0.1, 0.07, armID);  // 向下移动,抓取移动物体
//        baxter->jntCmdPub(currPos,armID);
    }

//    void ThirdTask::Ceshi3(const std::string &armID){
//        baxter->reSetArms(6);
//        std::vector<std::vector<double>> paths;
//        std::vector<double> currJnt = baxter->getJntPos(armID);  // 当前各个关节的角度
//        std::vector<double> currPos;  // 末端位姿
//        baxter->joint2Cart(currJnt, currPos, armID);  // Fk: obtain currPos 正解
//        int count = 0;
//        while (1)
//        {
//            std::vector<double> posHold_0 = {CirclePos[0], CirclePos[1], CirclePos[2], currPos[3], currPos[4],
//                                             currPos[5]};  // 后三位为欧拉角
//            std::vector<double> jntHold_0;
//            baxter->cart2Joints(currJnt, posHold_0, jntHold_0, armID);  // 逆解求得 jntHold_0
//            baxter->leftMoveOnce(jntHold_0);
//            count ++;
//            if (count > 50)
//                break;
//        }
//    }

    void ThirdTask::Ceshi4(const std::string &armID)
    {
        cv::FileStorage fs("/home/hit/ros_ws/offset.yml", cv::FileStorage::READ);
        cv::Mat offset;
        fs["offset"]>>offset;
        double cup1_x, cup1_y, cup1_z;
        std::vector<double> currPos;  // 末端位姿
        std::vector<std::vector<double>> paths;
        double next_step = 0.0;
        double endpoint_comp_x = 9.191; // x补偿
        double endpoint_comp_y = 9.191; // y补偿
        for (int i = 0; i < 64; ++i)
        {
            while (1)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    next_step = camera->gettestorderInfo();
                    if (next_step != 0)
                    {
                        std::cout << "已经接受到指令信息" << std::endl;
                        break;
                    }
                }
            std::vector<double> currJnt = baxter->getJntPos(armID);  // 当前各个关节的角度
            baxter->joint2Cart(currJnt, currPos, armID);  // Fk: obtain currPos 正解
            paths.clear();
            paths.push_back(currPos);
            currPos[0] = offset.at<double>(i,3) - endpoint_comp_x;
            currPos[1] = offset.at<double>(i,4) + endpoint_comp_y;
            currPos[2] = offset.at<double>(i,5);
            paths.push_back(currPos);
            baxter->moveTo(paths, 0.02, 0.01, 0.1, 0.07, armID);  //
            std::cout << "已经运动到指定点" << std::endl;
        }

//        for (int i = 0; i < 3; ++i)
//        {
//            std::vector<double> currJnt = baxter->getJntPos(armID);  // 当前各个关节的角度
//            std::vector<double> currPos;  // 末端位姿
//            baxter->joint2Cart(currJnt, currPos, armID);  // Fk: obtain currPos 正解
//            paths.clear();
//            paths.push_back(currPos);
//            currPos[0] += 0.10;
//            currPos[1] = 0.394;
//            currPos[2] = -0.087;
//            paths.push_back(currPos);
//            baxter->moveTo(paths, 0.02, 0.01, 0.1, 0.07, armID);  //
//            std::cout << "在x方向运动" << std::endl;
//
//            for (int j = 0; j < 3; ++j)
//            {
//                currJnt = baxter->getJntPos(armID);  // 当前各个关节的角度
//                baxter->joint2Cart(currJnt, currPos, armID);  // Fk: obtain currPos 正解
//                paths.clear();
//                paths.push_back(currPos);
//                currPos[1] -= 0.10;
//                currPos[2] = -0.087;
//                paths.push_back(currPos);
//                baxter->moveTo(paths, 0.02, 0.01, 0.1, 0.07, armID);  //
//                std::cout << "在y方向运动" << std::endl;
//                for (int k = 0; k < 3; ++k)
//                {
//                    while (1)
//                    {
//                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
//                        next_step = camera->gettestorderInfo();
////                        std::cout << " next_step ： " << next_step << std::endl;
//                        if (next_step != 0)
//                            break;
//                    }
//                    currJnt = baxter->getJntPos(armID);  // 当前各个关节的角度
//                    baxter->joint2Cart(currJnt, currPos, armID);  // Fk: obtain currPos 正解
//                    paths.clear();
//                    paths.push_back(currPos);
//                    currPos[2] += 0.10;
//                    paths.push_back(currPos);
//                    baxter->moveTo(paths, 0.02, 0.01, 0.1, 0.07, armID);  //
//                    std::cout << "已接受到信息在z方向运动" << std::endl;
//                    next_step = 0.0;
//                }
//            }
//        }
    }
}