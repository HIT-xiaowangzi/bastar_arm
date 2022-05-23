//
// Created by hit on 20-11-11.
//

#ifndef WRC_HIT_LIB_BAXTERCONTROLLER_H
#define WRC_HIT_LIB_BAXTERCONTROLLER_H

#include <time.h>
#include <math.h>
#include <ros/ros.h>

#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>
#include <sensor_msgs/Range.h>

#include "kdl/tree.hpp"

const clock_t INPUT_BLOCKING_TIME = CLOCKS_PER_SEC/2;

class BaxterController
{
  public:
  BaxterController(ros::NodeHandle nh);
  ~BaxterController();
  void reSetArms(int reSetType);
  void leftMoveOnce(const std::vector<double>& tarJnt);
  void rightMoveOnce(const std::vector<double>& tarJnt);
  void moveCart(const std::vector<double>& tarPos, const std::string &armID);

//    void jntCmdPub(std::vector<double> tarJnt, const std::string &armID);
    void leftJntCmdPub(const std::vector<double>& tarJnt);
    void rightJntCmdPub(const std::vector<double>& tarJnt);

    int joint2Cart(const std::vector<double> &Jnt, std::vector<double> &Cart, const std::string &armID);

  int cart2Joints(const std::vector<double> &initJoints, const std::vector<double> &targetPos, std::vector<double> &targetJoints, const std::string &armID);

  void gripperCallback(const baxter_core_msgs::EndEffectorStateConstPtr &msg);
  void irCallback(const sensor_msgs::RangeConstPtr &msg);
  void endpointCallback(const baxter_core_msgs::EndpointStateConstPtr &msg);
  void armjntCallback(const sensor_msgs::JointStateConstPtr& jnt_msg);

  float getRange();
  void getPosition(float position[]);
  void getOrientation(float orientation[]);
  std::vector<double> getJntPos(const std::string &armID);

  void grip(const std::string &armID);
  void release(const std::string &armID);
  void moveTo(const std::vector<std::vector<double>> pathInput, double rad, double eqrad, double vel, double acc, const std::string &armID);

    void start();
    float distanceToSetPosition();
    double leftDis2SetJnt(const std::vector<double>& setJnt_l, const std::vector<double>& currJnt_l);
    double rightDis2SetJnt(const std::vector<double>& setJnt_r, const std::vector<double>& currJnt_r);
  std::vector<double> getleftTor();
  std::vector<double> getRightTor();

  private:
  ros::NodeHandle nh;
  ros::Subscriber gripper_sub, ir_sub, endpoint_sub, jntposition_sub;
  ros::Publisher right_gripper_pub, left_gripper_pub, right_jnt_pub, left_jnt_pub;;
  ros::ServiceClient ik_client;
  baxter_core_msgs::JointCommand joint_cmd;
  double leftJntPos[7], rightJntPos[7];
  KDL::Tree baxterTree;
  KDL::Chain leftArmChain, rightArmChain;
  std::vector<double> lower, upper;

  clock_t last_input_time;
  unsigned int gripper_hid;
  float range;
  double position[3], set_position[3];
  double orientation[4], set_orientation[4];
  bool has_to_move;
  double leftTorPos[7];
  double rightTorPos[7];
};

#endif //WRC_HIT_LIB_BAXTERCONTROLLER_H
