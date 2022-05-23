//
// Created by hit on 20-11-11.
//

#include <thread>
#include <future>
#include <cmath>
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/path_roundedcomposite.hpp"
#include "kdl/rotational_interpolation_sa.hpp"
#include "kdl/velocityprofile.hpp"
#include "kdl/velocityprofile_trap.hpp"
#include "kdl/trajectory.hpp"
#include "kdl/trajectory_segment.hpp"
#include "kdl/trajectory_composite.hpp"
#include "kdl/trajectory_stationary.hpp"
#include "trac_ik/trac_ik.hpp"
#include <std_msgs/Float64.h>

#include "baxtercontroller.h"

#define L_ARM "left"
#define R_ARM "right"

BaxterController::BaxterController(ros::NodeHandle nh)
{
    gripper_hid = 0;
    has_to_move = false;
    last_input_time = clock();
    this->nh = nh;

    std::cout << std::setw(80) << std::left << "Registering arm joints callback: ";
    jntposition_sub = nh.subscribe("/robot/joint_states", 2, &BaxterController::armjntCallback, this);
    if(jntposition_sub == nullptr){
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::left << "\033[1;32m[OK]\033[0m" << std::endl;

    std::cout << std::setw(80) << std::left << "Registering gripper callback: ";
    gripper_sub = nh.subscribe("/robot/end_effector/right_gripper/state", 2, &BaxterController::gripperCallback, this);
    if(gripper_sub == nullptr)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;

    std::cout << std::setw(80) << std::left << "Registering two grippers' publisher: ";
    right_gripper_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", 2);
    left_gripper_pub  = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 2);

    if(right_gripper_pub == nullptr || left_gripper_pub == nullptr)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;

    std::cout << std::setw(80) << std::left << "Registering IR subscriber: ";
    ir_sub = nh.subscribe("/robot/range/right_hand_range/state", 2, &BaxterController::irCallback, this);
    if(ir_sub == nullptr)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;
    std::cout << std::setw(80) << std::left << "Registering endpoint subscriber: ";
    endpoint_sub = nh.subscribe("/robot/limb/right/endpoint_state", 2, &BaxterController::endpointCallback, this);
    if(endpoint_sub == nullptr)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;
    std::cout << std::setw(80) << std::left << "Registering inverse kinematic solver client: ";
    ik_client = nh.serviceClient<baxter_core_msgs::SolvePositionIK>("ExternalTools/right/PositionKinematicsNode/IKService");
    if(ik_client == nullptr)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;
    std::cout << std::setw(80) << std::left << "Registering joint publisher: ";

    left_jnt_pub = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 2);
    right_jnt_pub = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 2);

    if(right_jnt_pub == nullptr || left_jnt_pub == nullptr)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;
    
    kdl_parser::treeFromFile("/home/wangzirui/ros_ws/src/baxter_common/baxter_description/urdf/baxter.urdf", baxterTree);
    std::cout << std::setw(80) << std::left << "Registering left and right arm chain: ";
    bool l_valid = baxterTree.getChain("torso", "left_gripper", leftArmChain);
    bool r_valid = baxterTree.getChain("torso", "right_gripper", rightArmChain);
    if (!l_valid || !r_valid )
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;
    lower = {-1.7016, -2.147, -3.0541, -0.05, -3.059, -1.5707, -3.059};
    upper = { 1.7016,  1.2471,  3.0541,  2.618, 3.059,  2.094,   3.059};
}

BaxterController::~BaxterController()
{
}

void BaxterController::reSetArms(int reSetType)
{
    std::cout << std::right << "\033[1;31m[Setting Robot To Home Position]\033[0m" << std::endl;
    std::vector<double> homePosLeft, homePosRight;
    if (reSetType == 0) {  // 双臂展开
//        homePosLeft =  { 0.8, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50};
//        homePosRight = { -0.8, -1.0, 1.19, 1.94, -0.67, 1.03,  0.50};
//        homePosLeft =  { 0.8, -1.0, -1.19, 1.94,  0.67, 1.03, 0.20};
//        homePosRight = { -0.8, -1.0, 1.19, 1.94, -0.67, 1.03, -0.20};
        homePosLeft =  { 0.4897233665324183, -0.6511748444573582,-0.7834806874124751, 1.0680341235652193,
                         0.6369855221694181, 1.3575729972785913, 0.6055389160177671};
        homePosRight = {  -0.1457281748491143, -0.7742768026851626,0.42721364942608775, 1.3483691125512787,
                          -0.3236699462438223, 1.0895098545956152, -0.45674277959288195};
    } else if (reSetType == 1) {   // 双臂合在面前
        homePosLeft  = { 0.1, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50};
        homePosRight = {-0.1, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50};
    } else if (reSetType == 2) {   //
        homePosLeft  = {0, 0, 0, 0, 0, 0, 0};
        homePosRight = {0, 0, 0, 0, 0, 0, 0};
    } else if (reSetType == 3){    //
        homePosLeft  = {-0.553383569229663, 0.0030679615757708274, 0.11658253987929144, 1.5681118604158641,
                        0.26537867630417655, -1.5305293311126715, -0.02914563496982286};
        homePosRight = { 0.76, -0.8, 0.0, 2.0, 0.0, -1.0, 0.0};
    } else if (reSetType == 4){    // 抓取移动物体的初始状态(使用左臂,右臂转置另一边, 任务一)
        homePosLeft  = {-0.76, -0.8, 0.0, 2.0, 0.0, -1.0, 0.0};
        homePosRight = {-0.1457281748491143, -0.7742768026851626,0.42721364942608775, 1.3483691125512787,
                        -0.3236699462438223, 1.0895098545956152, -0.45674277959288195};
    } else if (reSetType == 5) {    // 抓取移动物体的初始状态(使用右臂,左臂转置另一边, 任务一)
        homePosLeft = {0.4897233665324183, -0.6511748444573582, -0.7834806874124751, 1.0680341235652193,
                       0.6369855221694181, 1.3575729972785913, 0.6055389160177671};
    } else if (reSetType == 6){    // 任务三, 左臂水平抓取宽口杯子, 右臂展开
        homePosLeft = {0.5445631796993219, -0.2626942099253771, -0.41379131753209036, 2.0428789142664, 1.9404856966750483,
                       -1.0312185846559694, 0.0279951493789088};
        homePosRight = {-0.1457281748491143, -0.7742768026851626,0.42721364942608775, 1.3483691125512787,
                        -0.3236699462438223, 1.0895098545956152, -0.45674277959288195};
    } else if (reSetType == 7)     // 任务三, 左臂水平, 右臂展开
    {
        homePosLeft = {-0.8981457513069098, -0.5238544390628688, 0.09318933286403888, 1.94662161982659,
                       -0.08091748656095557, -1.4258351423394922, -0.060975736318445196};   // 双臂水平夹持,水平对齐
        homePosRight = {-0.1457281748491143, -0.7742768026851626,0.42721364942608775, 1.3483691125512787,
                        -0.3236699462438223, 1.0895098545956152, -0.45674277959288195};
    }
      else if(reSetType==8)
      {
        homePosLeft={0.171806,-1.18807,-0.0245437,1.45997,0.0559903,1.3848,-0.080534};
        homePosRight={-0.171806,-1.18807,0.0245437,1.45997,-0.0559903,1.3848,0.080534};
      }
    else
        std::cout << std::right << "\033[1;31m[Home Position Error]\033[0m" << std::endl;
    auto thd1 = std::async(std::launch::async, &BaxterController::leftMoveOnce, this, homePosLeft);
    auto thd2 = std::async(std::launch::async, &BaxterController::rightMoveOnce, this, homePosRight);
    thd1.wait();
    thd2.wait();
    thd1 = std::async(std::launch::async, &BaxterController::release, this, L_ARM);
    thd2 = std::async(std::launch::async, &BaxterController::release, this, R_ARM);
    thd1.wait();
    thd2.wait();
    std::cout << std::right << "\033[1;32m[Setting Finished]\033[0m" << std::endl;
}

void BaxterController::leftMoveOnce(const std::vector<double>& tarJnt)
{
    std::vector<double> currJoint;
    auto startTime = std::chrono::high_resolution_clock::now();
    while (true)
    {
        leftJntCmdPub(tarJnt);
        currJoint = getJntPos(L_ARM);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        auto endTime=std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::ratio<1, 1> > duration_s(endTime - startTime); 
        if (leftDis2SetJnt(tarJnt, currJoint) < 0.03)
            break;
        if(duration_s.count()>5)
        {
            std::cout<<"Can't reach the expected place!"<<std::endl;
            break;
        }
    }
}

void BaxterController::rightMoveOnce(const std::vector<double>& tarJnt)
{
    std::vector<double> currJoint;
    auto startTime = std::chrono::high_resolution_clock::now();
    while (true)
    {
        rightJntCmdPub(tarJnt);
        currJoint = getJntPos(R_ARM);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        auto endTime=std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::ratio<1, 1> > duration_s(endTime - startTime);
        if (rightDis2SetJnt(tarJnt, currJoint) < 0.03)
            break;
        if(duration_s.count()>5)
        {
            std::cout<<"Can't reach the expected place!"<<std::endl;
            break;
        }
          
    }
}

void BaxterController::moveCart(const std::vector<double>& tarPos, const std::string &armID)
{
    std::vector<double> currJoint = getJntPos(armID);
    std::vector<double> tarJnt;
    int rc = cart2Joints(currJoint, tarPos, tarJnt, armID);
    if(rc > 0)
    {
        if (armID == L_ARM)
            leftMoveOnce(tarJnt);
        else
            rightMoveOnce(tarJnt);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

//void BaxterController::jntCmdPub(std::vector<double> tarJnt, const std::string &armID)
//{
//    baxter_core_msgs::JointCommand jointCmd;
//    jointCmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
//    std::string jointNamesArray[] = {armID+"_s0", armID+"_s1", armID+"_e0", armID+"_e1", armID+"_w0", armID+"_w1", armID+"_w2"};
//    int num_jnt = 7;
//    std::vector<std::string> jointNamesVector(num_jnt);
//    for( std::size_t j = 0; j < num_jnt; ++j)
//        jointNamesVector[j] = jointNamesArray[j];
//    jointCmd.names = jointNamesVector;
//    jointCmd.command.resize(num_jnt);
//    for( std::size_t j = 0; j < num_jnt; ++j)
//        jointCmd.command[j] = tarJnt[j];
//    if (armID == L_ARM)
//        left_jnt_pub.publish(jointCmd);
//    else if (armID == R_ARM)
//        right_jnt_pub.publish(jointCmd);
//}

void BaxterController::leftJntCmdPub(const std::vector<double>& tarJnt)
{
    baxter_core_msgs::JointCommand jointCmd;
    jointCmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    const std::string armID = L_ARM;
    std::string jointNamesArray[] = {armID+"_s0", armID+"_s1", armID+"_e0", armID+"_e1", armID+"_w0", armID+"_w1", armID+"_w2"};
    int num_jnt = 7;
    std::vector<std::string> jointNamesVector(num_jnt);
    for( std::size_t j = 0; j < num_jnt; ++j)
        jointNamesVector[j] = jointNamesArray[j];
    jointCmd.names = jointNamesVector;
    jointCmd.command.resize(num_jnt);
    for( std::size_t j = 0; j < num_jnt; ++j)
        jointCmd.command[j] = tarJnt[j];
    left_jnt_pub.publish(jointCmd);
}

void BaxterController::rightJntCmdPub(const std::vector<double>& tarJnt)
{
    baxter_core_msgs::JointCommand jointCmd;
    jointCmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    const std::string armID = R_ARM;
    std::string jointNamesArray[] = {armID+"_s0", armID+"_s1", armID+"_e0", armID+"_e1", armID+"_w0", armID+"_w1", armID+"_w2"};
    int num_jnt = 7;
    std::vector<std::string> jointNamesVector(num_jnt);
    for( std::size_t j = 0; j < num_jnt; ++j)
        jointNamesVector[j] = jointNamesArray[j];
    jointCmd.names = jointNamesVector;
    jointCmd.command.resize(num_jnt);
    for( std::size_t j = 0; j < num_jnt; ++j)
        jointCmd.command[j] = tarJnt[j];
    right_jnt_pub.publish(jointCmd);
}

int BaxterController::joint2Cart(const std::vector<double> &Jnt, std::vector<double> &Cart, const std::string &armID)  // 正解
{
    unsigned int nj = Jnt.size();
    KDL::JntArray jntValue(nj);
     for (int j = 0; j < nj; ++j)
      jntValue(j) = Jnt[j];
    KDL::Frame EEFrame;
    if (armID == L_ARM){
      KDL::ChainFkSolverPos_recursive fk_solver(leftArmChain);
      fk_solver.JntToCart(jntValue, EEFrame);
    } else if (armID == R_ARM) {
      KDL::ChainFkSolverPos_recursive fk_solver(rightArmChain);
      fk_solver.JntToCart(jntValue, EEFrame);
    }
    double alfa, beta, gamma;
    EEFrame.M.GetRPY(alfa, beta, gamma);
    Cart = {EEFrame.p[0], EEFrame.p[1], EEFrame.p[2], alfa, beta, gamma};
    return 0;
}

int BaxterController::cart2Joints(const std::vector<double> &initJnt, const std::vector<double> &targetPos,
                                  std::vector<double> &tarJnt, const std::string &armID,TRAC_IK::SolveType Type,
                                  std::vector<double> lower2,std::vector<double> upper2)   // 逆解
{
  unsigned int nj = initJnt.size();
  KDL::JntArray lower_limits(nj), upper_limits(nj);
  for (int j = 0; j < nj; ++j)
  {
    lower_limits(j) = lower2[j];
    upper_limits(j) = upper2[j];
  }
  double timeout = 0.010;//_in_secs
  double error = 1e-5;
  KDL::Vector _vel{error, error, error};
  KDL::Vector _rot{error, error, error};
  KDL::Twist tolerances(_vel, _rot);

  TRAC_IK::SolveType type =Type;//TRAC_IK::Speed
//    if(_Type == 2)
//        type = TRAC_IK::Distance;
  KDL::JntArray joint_seed(nj);
  for (int l = 0; l < nj; ++l)
    joint_seed(l) = initJnt[l];

  KDL::Rotation F_Rot = KDL::Rotation::RPY(targetPos[3], targetPos[4], targetPos[5]);
  KDL::Vector F_V = {targetPos[0], targetPos[1], targetPos[2]};
  KDL::Frame end_effector_pose(F_Rot, F_V);
  KDL::JntArray return_joints(nj);
  int rc = -1;
  if (armID == L_ARM){
    TRAC_IK::TRAC_IK ik_solver(leftArmChain, lower_limits, upper_limits, timeout, error, type);
    rc = ik_solver.CartToJnt(joint_seed, end_effector_pose, return_joints, tolerances);
  } else if (armID == R_ARM){
    TRAC_IK::TRAC_IK ik_solver(rightArmChain, lower_limits, upper_limits, timeout, error, type);
    rc = ik_solver.CartToJnt(joint_seed, end_effector_pose, return_joints, tolerances);
  }
  tarJnt.clear();
  if(rc >= 0){
    //printf("%s \n","Success get a IK solution using trac_IK!");
    for (int k = 0; k < nj; ++k)
      tarJnt.push_back(return_joints.data[k]);
    return 1;
  } else{
    std::cout << "\033[1;31mInverse kinematic solver found no solution for that movement\033[0m" << std::endl;
    return -1;
  }
}

void BaxterController::gripperCallback(const baxter_core_msgs::EndEffectorStateConstPtr &msg)
{
  if(gripper_hid == 0)
  {
    gripper_hid = msg->id;
    baxter_core_msgs::EndEffectorCommand cmd;
    cmd.id = gripper_hid;
    cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
    right_gripper_pub.publish(cmd);
  }
}

void BaxterController::irCallback(const sensor_msgs::RangeConstPtr &msg)
{
  this->range = msg->range;
}

void BaxterController::endpointCallback(const baxter_core_msgs::EndpointStateConstPtr &msg)
{
  position[0] = msg->pose.position.x;
  position[1] = msg->pose.position.y;
  position[2] = msg->pose.position.z;
  orientation[0] = msg->pose.orientation.x;
  orientation[1] = msg->pose.orientation.y;
  orientation[2] = msg->pose.orientation.z;
  orientation[3] = msg->pose.orientation.w;
  if(has_to_move)
    right_jnt_pub.publish(joint_cmd);
}

void BaxterController::armjntCallback(const sensor_msgs::JointStateConstPtr& jnt_msg)
{
    /* In the jointstate topic, the order of joint angles is e->s->w,  *
     * * while TRAC_IK requires the order of s->e->w.                    */
    leftJntPos[0] = jnt_msg->position[4];
    leftJntPos[1] = jnt_msg->position[5];
    leftJntPos[2] = jnt_msg->position[2];
    leftJntPos[3] = jnt_msg->position[3];
    leftJntPos[4] = jnt_msg->position[6];
    leftJntPos[5] = jnt_msg->position[7];
    leftJntPos[6] = jnt_msg->position[8];
    rightJntPos[0] = jnt_msg->position[11];
    rightJntPos[1] = jnt_msg->position[12];
    rightJntPos[2] = jnt_msg->position[9];
    rightJntPos[3] = jnt_msg->position[10];
    rightJntPos[4] = jnt_msg->position[13];
    rightJntPos[5] = jnt_msg->position[14];
    rightJntPos[6] = jnt_msg->position[15];
    leftTorPos[0] = jnt_msg->effort[4];
    leftTorPos[1] = jnt_msg->effort[5];
    leftTorPos[2] = jnt_msg->effort[2];
    leftTorPos[3] = jnt_msg->effort[3];
    leftTorPos[4] = jnt_msg->effort[6];
    leftTorPos[5] = jnt_msg->effort[7];
    leftTorPos[6] = jnt_msg->effort[8];
    rightTorPos[0]=jnt_msg->effort[11];
    rightTorPos[1]=jnt_msg->effort[12];
    rightTorPos[2]=jnt_msg->effort[9];
    rightTorPos[3]=jnt_msg->effort[10];
    rightTorPos[4]=jnt_msg->effort[13];
    rightTorPos[5]=jnt_msg->effort[14];
    rightTorPos[6]=jnt_msg->effort[15];
}

float BaxterController::getRange()
{
  return range;
}

void BaxterController::getPosition(float position[])
{
  for(int i = 0; i < 3; i++)
    position[i] = this->position[i];
}

void BaxterController::getOrientation(float orientation[])
{
  for(int i = 0; i < 4; i++)
    orientation[i] = this->orientation[i];
}

std::vector<double> BaxterController::getleftTor()
{
    std::vector<double>ans;
    ans.clear();
    for(int i=0;i<7;i++)
    {
        ans.push_back(this->leftTorPos[i]);
    }
    return ans;
}

std::vector<double> BaxterController::getRightTor()
{
    std::vector<double>ans;
    ans.clear();
    for(int i=0;i<7;i++)
    {
        ans.push_back(this->rightTorPos[i]);
    }
    return ans;
}
std::vector<double> BaxterController::getJntPos(const std::string &armID)
{
  std::vector<double> jntPos;
  bool valid = false;
  jntPos.resize(7);
  while (!valid)
  {
    if (armID == L_ARM) {
      for(int i = 0; i < 7; i++)
        jntPos[i] = this->leftJntPos[i];
    } else if (armID == R_ARM) {
      for(int i = 0; i < 7; i++)
        jntPos[i] = this->rightJntPos[i];
    }
    int jnt_limit_test = 0;
    int jnt_zero_test = 0;
    for (int j = 0; j < 7; ++j)
    {
      if (jntPos[j] > lower[j] && jntPos[j] < upper[j])
        jnt_limit_test++;
      if (std::fabs(jntPos[j]) < 1e-7)
        jnt_zero_test++;
    }
//    std::cout << " joint test :" << jnt_limit_test<< ", " << jnt_zero_test << std::endl;
    if (jnt_limit_test == 7 && jnt_zero_test == 0)
      valid = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return jntPos;
}

void BaxterController::grip(const std::string &armID)
{
    baxter_core_msgs::EndEffectorCommand cmd;
    cmd.id = gripper_hid;
    cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_GRIP;
    if (armID == L_ARM)
        left_gripper_pub.publish(cmd);
    else if (armID == R_ARM)
        right_gripper_pub.publish(cmd);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << std::left << "\033[1;32m[Hand grip finished]\033[0m" << std::endl;
}

void BaxterController::release(const std::string &armID)
{
  baxter_core_msgs::EndEffectorCommand cmd;
  cmd.id = gripper_hid;
  cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_RELEASE;
    if (armID == L_ARM)
        left_gripper_pub.publish(cmd);
    else if (armID == R_ARM)
        right_gripper_pub.publish(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << std::left << "\033[1;32m[Hand release finished]\033[0m" << std::endl;
}

void BaxterController::moveTo(const std::vector<std::vector<double>> pathInput, double rad, double eqrad, double vel, double acc, const std::string &armID) {
  std::cout << std::right << "\033[1;31m[MoveTo Running]\033[0m" << std::endl;
  const int path_num = pathInput.size();
  auto *path = new KDL::Path_RoundedComposite(rad, eqrad, new KDL::RotationalInterpolation_SingleAxis());
  for (auto i : pathInput)
    path->Add(KDL::Frame(KDL::Rotation::RPY(i[3], i[4], i[5]), KDL::Vector(i[0], i[1], i[2])));
  path->Finish();
  KDL::VelocityProfile* velpref = new KDL::VelocityProfile_Trap(vel, acc);
  velpref->SetProfile(0, path->PathLength());
  KDL::Trajectory* traject = new KDL::Trajectory_Segment(path, velpref);
  auto *ctraject = new KDL::Trajectory_Composite();
  ctraject->Add(traject);
  ctraject->Add(new KDL::Trajectory_Stationary(1.0, KDL::Frame(
          KDL::Rotation::RPY(pathInput[path_num-1][3],pathInput[path_num-1][4],pathInput[path_num-1][5]),
          KDL::Vector(pathInput[path_num-1][0],pathInput[path_num-1][1],pathInput[path_num-1][2]))));

  std::chrono::high_resolution_clock::time_point _T0, _T;
  _T0 = _T = std::chrono::high_resolution_clock::now();
  double _TCount = std::chrono::duration_cast<std::chrono::duration<double> > (_T - _T0).count();
  KDL::Frame current_pose;
  double alfa, beta, gamma;
  std::vector<double> xyzRpy;
  std::vector<double> initJnt = getJntPos(armID);
  std::vector<double> tarJnt;
  int rc;
  baxter_core_msgs::JointCommand jointCmd;
  std::vector<std_msgs::Float64> pos_cmd;
  while (traject->Duration() > _TCount)
  {
    _TCount = std::chrono::duration_cast<std::chrono::duration<double> > (_T - _T0).count();
    current_pose = traject->Pos(_TCount);
    current_pose.M.GetRPY(alfa, beta, gamma);
    xyzRpy = {current_pose.p[0], current_pose.p[1], current_pose.p[2], alfa, beta, gamma};
    rc = cart2Joints(initJnt, xyzRpy, tarJnt, armID);
    if(rc > 0)
    {
        if (armID == L_ARM)
            leftJntCmdPub(tarJnt);
            //leftMoveOnce(tarJnt);
        else
            rightJntCmdPub(tarJnt);
            //rightMoveOnce(tarJnt);
        initJnt = tarJnt;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    _T = std::chrono::high_resolution_clock::now();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << std::left << "\033[1;32m[MoveTo Finished]\033[0m" << std::endl;
}

void BaxterController::AsyncmoveTo(const std::vector<std::vector<double>> pathInput,
std::vector<double> leftinitPos,std::vector<double> rightinitPos, double rad, double eqrad, double vel, double acc, const std::string &armID) {//两手同步的控制器
  //Created by wangzirui on 2022-5-22
  std::cout << std::right << "\033[1;31m[AsyncMoveTo Running]\033[0m" << std::endl;
  const int path_num = pathInput.size();
  auto *path = new KDL::Path_RoundedComposite(rad, eqrad, new KDL::RotationalInterpolation_SingleAxis());
  for (auto i : pathInput)
    path->Add(KDL::Frame(KDL::Rotation::RPY(i[3], i[4], i[5]), KDL::Vector(i[0], i[1], i[2])));
  path->Finish();
  KDL::VelocityProfile* velpref = new KDL::VelocityProfile_Trap(vel, acc);
  velpref->SetProfile(0, path->PathLength());
  KDL::Trajectory* traject = new KDL::Trajectory_Segment(path, velpref);
  auto *ctraject = new KDL::Trajectory_Composite();
  ctraject->Add(traject);
  ctraject->Add(new KDL::Trajectory_Stationary(1.0, KDL::Frame(
          KDL::Rotation::RPY(pathInput[path_num-1][3],pathInput[path_num-1][4],pathInput[path_num-1][5]),
          KDL::Vector(pathInput[path_num-1][0],pathInput[path_num-1][1],pathInput[path_num-1][2]))));

  std::chrono::high_resolution_clock::time_point _T0, _T;
  _T0 = _T = std::chrono::high_resolution_clock::now();
  double _TCount = std::chrono::duration_cast<std::chrono::duration<double> > (_T - _T0).count();
  KDL::Frame current_pose;
  double alfa, beta, gamma;
  std::vector<double> tarPos;
  std::vector<double> tarJnt2;
  std::vector<double> xyzRpy;
  std::vector<double> initJnt = getJntPos(armID);
  std::vector<double> tarJnt;
  int rc;
  baxter_core_msgs::JointCommand jointCmd;
  std::vector<std_msgs::Float64> pos_cmd;
  while (traject->Duration() > _TCount)
  {
    _TCount = std::chrono::duration_cast<std::chrono::duration<double> > (_T - _T0).count();
    current_pose = traject->Pos(_TCount);
    current_pose.M.GetRPY(alfa, beta, gamma);
    xyzRpy = {current_pose.p[0], current_pose.p[1], current_pose.p[2], alfa, beta, gamma};
    rc = cart2Joints(initJnt, xyzRpy, tarJnt, armID);
    if(rc > 0)
    {
        if (armID == L_ARM)
        {
          leftJntCmdPub(tarJnt);
          for(int i=0;i<6;i++)
          {
            tarPos.push_back(xyzRpy[i]+rightinitPos[i]-leftinitPos[i]);
          }
          cart2Joints(getJntPos(R_ARM),tarPos,tarJnt2,R_ARM);
          rightJntCmdPub(tarJnt2);
        }
        else
        {
          rightJntCmdPub(tarJnt);
          for(int i=0;i<6;i++)
          {
            tarPos.push_back(xyzRpy[i]+leftinitPos[i]-rightinitPos[i]);
          }
          cart2Joints(getJntPos(L_ARM),tarPos,tarJnt2,L_ARM);
          leftJntCmdPub(tarJnt2);
        }
        initJnt = tarJnt;
        tarPos.clear();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    _T = std::chrono::high_resolution_clock::now();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << std::left << "\033[1;32m[MoveTo Finished]\033[0m" << std::endl;
}

void BaxterController::start()
{
    has_to_move = true;
}

double BaxterController::leftDis2SetJnt(const std::vector<double>& setJnt_l, const std::vector<double>& currJnt_l)
{
  double jd = 0;
  for(int i = 0; i < setJnt_l.size(); i++)
    jd += std::fabs(setJnt_l[i] - currJnt_l[i]);
//    std::cout << jd << " jd" << std::endl;
  return jd;
}

double BaxterController::rightDis2SetJnt(const std::vector<double>& setJnt_r, const std::vector<double>& currJnt_r)
{
    double jd = 0;
    for(int i = 0; i < setJnt_r.size(); i++)
        jd += std::fabs(setJnt_r[i] - currJnt_r[i]);
//    std::cout << jd << " jd" << std::endl;
    return jd;
}












