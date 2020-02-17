#ifndef COMMUNICATE_MASTER_H
#define COMMUNICATE_MASTER_H

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>

#include <string>
#include <vector>
#include <queue>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>


namespace communicate_master
{
class CommunicateMaster
{
public:
  CommunicateMaster(ros::NodeHandle& nh);
  ~CommunicateMaster();
  bool moveRobot();
private:
//   void* waitReceiveThread(void* p_param);
  void receiveValue(const geometry_msgs::Vector3::ConstPtr& msg);
private:
  ros::NodeHandle nh_;
  ros::ServiceClient moving_service_;
  ros::ServiceClient gripper_activate_service_;
  ros::ServiceClient gripper_open_service_;
  ros::ServiceClient gripper_close_service_;
  ros::ServiceClient gripper_set_position_service_;
  ros::Subscriber object_pos_sub_;
  std::queue<std::vector<double> > work_order_;
  double grasp_offset_;
  double grasp_close_pos_;
};
} // namespace communicate_master

#endif // COMMUNICATE_MASTER_H
