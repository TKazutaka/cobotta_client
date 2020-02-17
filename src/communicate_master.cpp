#include <cobotta_client/communicate_master.h>

#include <iostream>

#include <geometry_msgs/PoseStamped.h>

#include <denso_state_srvs/Moving.h>
#include <denso_gripper_srvs/ActivateGripper.h>
#include <denso_gripper_srvs/Move.h>
#include <denso_gripper_srvs/SetPosition.h>

static const double WORK_OFFSET = 0.05;

using communicate_master::CommunicateMaster;

// Constructor
CommunicateMaster::CommunicateMaster(ros::NodeHandle& nh) : nh_(nh)
{
  ros::param::param<double>("~grasp_offset", grasp_offset_, 0.1); // offset for grasping
  ros::param::param<double>("~grasp_close_pos", grasp_close_pos_, 15); // gripper close range for grasping
  moving_service_ = nh.serviceClient<denso_state_srvs::Moving>("/state_behavior/moving");
  gripper_activate_service_ = nh.serviceClient<denso_gripper_srvs::ActivateGripper>("/parallel_gripper/activate");
  gripper_open_service_ = nh.serviceClient<denso_gripper_srvs::Move>("/parallel_gripper/open");
  gripper_close_service_ = nh.serviceClient<denso_gripper_srvs::Move>("/parallel_gripper/close");
  gripper_set_position_service_ = nh.serviceClient<denso_gripper_srvs::SetPosition>("/parallel_gripper/set_position");
  object_pos_sub_ = nh.subscribe("/object_position", 10, &CommunicateMaster::receiveValue, this);

  std::cout << "[SUCCESS] Initialize CommunicateMaster class !! " << std::endl;
}

// Destractor
CommunicateMaster::~CommunicateMaster()
{
  std::cout << "[FINISH] Finish !!" << std::endl;
}

void CommunicateMaster::receiveValue(const geometry_msgs::Vector3::ConstPtr& msg)
{
  std::cout << "Receive !!" << std::endl;

  std::vector<double> object_pose_vec;
  object_pose_vec.clear();
  object_pose_vec.push_back(msg->x);
  object_pose_vec.push_back(msg->y);
  object_pose_vec.push_back(msg->z);
  work_order_.push(object_pose_vec);
  
}

// Function for moveing robot
bool CommunicateMaster::moveRobot()
{

  std::cout << "\n======================================="
            << "\n    Receive object pose from master    "
            << "\n======================================="
            << std::endl;

  if (work_order_.empty())
  {
    std::cout << "\n[INFO] Not work order from master !!" << std::endl;
    return true;
  }

  std::vector<double> object_pose;
  object_pose.clear();
  std::cout << "[before pop] queue size: " << work_order_.size() << std::endl;
  object_pose = work_order_.front();
  work_order_.pop();
  std::cout << "[after pop] queue size: " << work_order_.size() << std::endl;

  std::cout << "\n[SUCCESS] Complete to receive object pose !! " << std::endl;

  std::cout << "\n====================================="
            << "\n      Activate parallel gripper      "
            << "\n====================================="
            << std::endl;

  // Activate parallel gripper
  denso_gripper_srvs::ActivateGripper activate_srv;
  if (!gripper_activate_service_.call(activate_srv))
  {
    ROS_ERROR("Failed to activate parallel gripper !!");
    return false;
  }

  ros::Duration(0.1).sleep();

  // Open Gripper for initialize
  denso_gripper_srvs::Move gripper_move_srv;
  if (!gripper_open_service_.call(gripper_move_srv))
  {
    ROS_ERROR("Failed to open gripper !!");
    return false;
  }

  std::cout << "\n[SUCCESS] Complete to activate parallel gripper !! " << std::endl;

  ros::Duration(0.1).sleep();

  std::cout << "\n========================================="
            << "\n   Robot move to grasp object position   "
            << "\n========================================="
            << std::endl;

  // Define robot position for grasping object
  geometry_msgs::PoseStamped grasp_position;
  std::cout << "object_pose 0 : " << object_pose[0] << std::endl;
  std::cout << "object_pose 1 : " << object_pose[1] << std::endl;
  std::cout << "object_pose 2 : " << object_pose[2] << std::endl;
  grasp_position.pose.position.x = object_pose[0];
  grasp_position.pose.position.y = object_pose[1];
  grasp_position.pose.position.z = object_pose[2]; // initialize
  grasp_position.pose.orientation.x = 1;
  grasp_position.pose.orientation.y = 0;
  grasp_position.pose.orientation.z = 0;
  grasp_position.pose.orientation.w = 0;

  // Moving robot to the position for grasping object
  denso_state_srvs::Moving move_srv;
  grasp_position.pose.position.z = object_pose[2] + grasp_offset_ + WORK_OFFSET;
  move_srv.request.target_pose = grasp_position;
  if (!moving_service_.call(move_srv))
  {
    ROS_ERROR("Failed to move robot !!");
    return false;
  }

  ros::Duration(0.1).sleep();

  // Moving robot to the position for grasping object
  grasp_position.pose.position.z = object_pose[2] + grasp_offset_;
  move_srv.request.target_pose = grasp_position;
  if (!moving_service_.call(move_srv))
  {
    ROS_ERROR("Failed to move robot !!");
    return false;
  }

  ros::Duration(0.1).sleep();

  std::cout << "\n[SUCCESS] Complete to move to grasp object position !! " << std::endl;

  std::cout << "\n=============================================="
            << "\n  Close parallel gripper for grasping object  "
            << "\n=============================================="
            << std::endl;

  denso_gripper_srvs::SetPosition set_pos_srv_;
  set_pos_srv_.request.position = grasp_close_pos_;
  if (!gripper_set_position_service_.call(set_pos_srv_))
  {
    ROS_ERROR("Failed to close gripper !!");
    return false;
  }

  std::cout << "\n[SUCCESS] Complete to grasp object !! " << std::endl;

  ros::Duration(0.1).sleep();

  std::cout << "\n===================================="
            << "\n   Robot move to transport object   "
            << "\n===================================="
            << std::endl;

  // Define robot position for transporting object
  geometry_msgs::PoseStamped transport_position;
  transport_position.pose.position.x = 0.111176; // TODO transport position x
  transport_position.pose.position.y = -0.218825; // TODO transport position y
  transport_position.pose.position.z = 0.1111; // TODO transport position z
  transport_position.pose.orientation.x = 1;
  transport_position.pose.orientation.y = 0;
  transport_position.pose.orientation.z = 0;
  transport_position.pose.orientation.w = 0;

  // Moving robot to the position for transporting object
  transport_position.pose.position.z += WORK_OFFSET; // TODO add transport position z
  move_srv.request.target_pose = transport_position;
  if (!moving_service_.call(move_srv))
  {
    ROS_ERROR("Failed to move robot !!");
    return false;
  }

  ros::Duration(0.1).sleep();

  transport_position.pose.position.z -= WORK_OFFSET; // TODO add transport position z
  move_srv.request.target_pose = transport_position;
  if (!moving_service_.call(move_srv))
  {
    ROS_ERROR("Failed to move robot !!");
    return false;
  }

  ros::Duration(0.1).sleep();

  std::cout << "\n[SUCCESS] Complete to transport object !! " << std::endl;

  std::cout << "\n=============================================="
            << "\n   Open parallel gripper for release object   "
            << "\n=============================================="
            << std::endl;

  // Open parallel gripper for releasing object
  if (!gripper_open_service_.call(gripper_move_srv))
  {
    ROS_ERROR("Failed to open gripper !!");
    return false;
  }

  std::cout << "\n[SUCCESS] Complete to release object !! " << std::endl;

  ros::Duration(0.1).sleep();

  // Define robot default position
  geometry_msgs::PoseStamped default_position;
  default_position.pose.position.x = -0.034074;
  default_position.pose.position.y = -0.070623;
  default_position.pose.position.z = 0.34166;
  default_position.pose.orientation.x = 0.46538;
  default_position.pose.orientation.y = 0.0057988;
  default_position.pose.orientation.z = 0.81983;
  default_position.pose.orientation.w =0.33357;

  std::cout << "\n===================================="
            << "\n   Robot move to initial position   "
            << "\n===================================="
            << std::endl;

  // Moving robot to the default position
  move_srv.request.target_pose = default_position;
  if (!moving_service_.call(move_srv))
  {
    ROS_ERROR("Failed to move robot !!");
    return false;
  }

  std::cout << "\n[SUCCESS] Complete to move to initial position !! " << std::endl;

  std::cout << "\n*************************************"
            << "\n**********  Success work  ***********"
            << "\n*************************************"
            << std::endl;

  return true;
}
