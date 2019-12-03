#ifndef INITIAL_POSE_PUBLISHER_H_
#define INITIAL_POSE_PUBLISHER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Empty.h>

namespace initial_pose_publisher
{
class InitialPosePublisher
{
public:
  InitialPosePublisher();
  void run();

private:
  ros::NodeHandle private_nh_;
  ros::Publisher initial_pose_publisher_;
  geometry_msgs::PoseWithCovarianceStamped pose_to_publish_;
  bool wait_for_publish_;
};

}  // namespace initial_pose_publisher

#endif  // INITIAL_POSE_PUBLISHER_H_