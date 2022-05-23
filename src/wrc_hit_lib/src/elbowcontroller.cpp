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

#include "elbowcontroller.h"

#define L_ARM "left"
#define R_ARM "right"

ElbowController::ElbowController(ros::NodeHandle nh)
{
    gripper_hid = 0;
    has_to_move = false;
    //last_input_time = clock();
    this->nh = nh;

    jntposition_sub = nh.subscribe("/robot/joint_states", 1000, &ElbowController::armjntCallback, this);
    ir_sub = nh.subscribe("/robot/range/right_hand_range/state", 2, &ElbowController::irCallback, this);
    endpoint_sub = nh.subscribe("/robot/limb/right/endpoint_state", 2, &ElbowController::endpointCallback, this);
    ik_client = nh.serviceClient<baxter_core_msgs::SolvePositionIK>("ExternalTools/right/PositionKinematicsNode/IKService");
    left_jnt_pub = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 2);
    right_jnt_pub = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 2);
    kdl_parser::treeFromFile("/home/wangzirui/ros_ws/src/baxter_common/baxter_description/urdf/baxter.urdf", baxterTree);
    bool l_elbow = baxterTree.getChain("torso", "left_lower_forearm",leftElbowChain);
    bool r_elbow = baxterTree.getChain("torso", "right_lower_forearm",rightElbowChain);
    if(l_elbow &&r_elbow)
    {
      std::cout<<"Successfully create elbow chain!"<<std::endl;
    }
    else
    {
      std::cout<<"Fail to create elbow chain!"<<std::endl;\
      return ;
    }
    lower = {-1.7016, -2.147, -3.0541, -0.05, -3.059, -1.5707, -3.059};
    upper = { 1.7016,  1.0471,  3.0541,  2.618, 3.059,  2.094,   3.059};
}

ElbowController::~ElbowController()
{
}

void ElbowController::leftMoveOnce(const std::vector<double>& tarJnt)
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

void ElbowController::rightMoveOnce(const std::vector<double>& tarJnt)
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

void ElbowController::moveCart(const std::vector<double>& tarPos, const std::string &armID)
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

void ElbowController::leftJntCmdPub(const std::vector<double>& tarJnt)
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
    for( std::size_t j = 0; j < 6; ++j)
        jointCmd.command[j] = tarJnt[j];
    jointCmd.command[6]=0;//最后一个关节写成定值
    left_jnt_pub.publish(jointCmd);
}

void ElbowController::rightJntCmdPub(const std::vector<double>& tarJnt)
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
    for( std::size_t j = 0; j < 6; ++j)
        jointCmd.command[j] = tarJnt[j];
    jointCmd.command[6]=0;//最后一个关节也写成定值
    right_jnt_pub.publish(jointCmd);
}

int ElbowController::joint2Cart(const std::vector<double> &Jnt, std::vector<double> &Cart, const std::string &armID)  // 正解
{
    unsigned int nj = Jnt.size();
    KDL::JntArray jntValue(6);
     for (int j = 0; j < 6; ++j)
      jntValue(j) = Jnt[j];
    KDL::Frame EEFrame;
    if (armID == L_ARM){
      KDL::ChainFkSolverPos_recursive fk_solver(leftElbowChain);
      fk_solver.JntToCart(jntValue, EEFrame);
    } else if (armID == R_ARM) {
      KDL::ChainFkSolverPos_recursive fk_solver(rightElbowChain);
      fk_solver.JntToCart(jntValue, EEFrame);
    }
    double alfa, beta, gamma;
    EEFrame.M.GetRPY(alfa, beta, gamma);
    Cart = {EEFrame.p[0], EEFrame.p[1], EEFrame.p[2], alfa, beta, gamma};
    return 0;
}

int ElbowController::cart2Joints(const std::vector<double> &initJnt, const std::vector<double> &targetPos,
                                  std::vector<double> &tarJnt, const std::string &armID)   // 逆解
{
  unsigned int nj = initJnt.size();
  KDL::JntArray lower_limits(nj), upper_limits(nj);
  for (int j = 0; j < nj; ++j)
  {
    lower_limits(j) = lower[j];
    upper_limits(j) = upper[j];
  }
  double timeout = 0.007;//_in_secs
  double error = 1e-5;
  KDL::Vector _vel{error, error, error};
  KDL::Vector _rot{error, error, error};
  KDL::Twist tolerances(_vel, _rot);

  TRAC_IK::SolveType type = TRAC_IK::Distance;//TRAC_IK::Speed
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
    TRAC_IK::TRAC_IK ik_solver(leftElbowChain, lower_limits, upper_limits, timeout, error, type);
    rc = ik_solver.CartToJnt(joint_seed, end_effector_pose, return_joints, tolerances);
  } else if (armID == R_ARM){
    TRAC_IK::TRAC_IK ik_solver(rightElbowChain, lower_limits, upper_limits, timeout, error, type);
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


void ElbowController::irCallback(const sensor_msgs::RangeConstPtr &msg)
{
  //std::cout<<"3"<<std::endl;
  this->range = msg->range;
}

void ElbowController::endpointCallback(const baxter_core_msgs::EndpointStateConstPtr &msg)
{
  //std::cout<<"1"<<std::endl;
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

void ElbowController::armjntCallback(const sensor_msgs::JointStateConstPtr& jnt_msg)
{
    /* In the jointstate topic, the order of joint angles is e->s->w,  *
     * * while TRAC_IK requires the order of s->e->w.*/
    //std::cout<<"2"<<std::endl;
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
}

float ElbowController::getRange()
{
  return range;
}

void ElbowController::getPosition(float position[])
{
  for(int i = 0; i < 3; i++)
    position[i] = this->position[i];
}

void ElbowController::getOrientation(float orientation[])
{
  for(int i = 0; i < 4; i++)
    orientation[i] = this->orientation[i];
}

std::vector<double> ElbowController::getleftTor()
{
    std::vector<double>ans;
    ans.clear();
    for(int i=0;i<7;i++)
    {
        ans.push_back(this->leftTorPos[i]);
    }
    return ans;
}

std::vector<double> ElbowController::getRightTor()
{
    std::vector<double>ans;
    ans.clear();
    for(int i=0;i<7;i++)
    {
        ans.push_back(this->rightTorPos[i]);
    }
    return ans;
}
std::vector<double> ElbowController::getJntPos(const std::string &armID)
{
  std::vector<double> jntPos;
  bool valid = false;
  jntPos.resize(6);//最后一个手腕的数据不需要了
  while (!valid)
  {
    if (armID == L_ARM) {
      for(int i = 0; i < 6; i++)
        jntPos[i] = this->leftJntPos[i];
    } else if (armID == R_ARM) {
      for(int i = 0; i < 6; i++)
        jntPos[i] = this->rightJntPos[i];
    }
    int jnt_limit_test = 0;
    int jnt_zero_test = 0;
    for (int j = 0; j < 6; ++j)
    {
      if (jntPos[j] > lower[j] && jntPos[j] < upper[j])
        jnt_limit_test++;
      if (std::fabs(jntPos[j]) < 1e-7)
        jnt_zero_test++;
    }
    if (jnt_limit_test == 6 && jnt_zero_test == 0)
    {
      valid = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  return jntPos;
}


void ElbowController::moveTo(const std::vector<std::vector<double>> pathInput, double rad, double eqrad, double vel, double acc, const std::string &armID) {
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
        else
            rightJntCmdPub(tarJnt);
        initJnt = tarJnt;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    _T = std::chrono::high_resolution_clock::now();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << std::left << "\033[1;32m[MoveTo Finished]\033[0m" << std::endl;
}

void ElbowController::start()
{
    has_to_move = true;
}

double ElbowController::leftDis2SetJnt(const std::vector<double>& setJnt_l, const std::vector<double>& currJnt_l)
{
  double jd = 0;
  for(int i = 0; i < currJnt_l.size(); i++)
    jd += std::fabs(setJnt_l[i] - currJnt_l[i]);
//    std::cout << jd << " jd" << std::endl;
  return jd;
}

double ElbowController::rightDis2SetJnt(const std::vector<double>& setJnt_r, const std::vector<double>& currJnt_r)
{
    double jd = 0;
    for(int i = 0; i < currJnt_r.size(); i++)
        jd += std::fabs(setJnt_r[i] - currJnt_r[i]);
//    std::cout << jd << " jd" << std::endl;
    return jd;
}












