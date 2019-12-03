/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "initial_pose_publisher/initial_pose_publisher.h"
#include <cmath>
#include <string>
#include <tf2/LinearMath/Quaternion.h>

namespace initial_pose_publisher
{
InitialPosePublisher::InitialPosePublisher() : private_nh_("~"), wait_for_publish_(false)
{
  // Get parameters
  // header
  pose_to_publish_.header.frame_id = private_nh_.param<std::string>("frame_id", "world");
  // position
  pose_to_publish_.pose.pose.position.x = private_nh_.param<double>("pose_x", 0.0);
  pose_to_publish_.pose.pose.position.y = private_nh_.param<double>("pose_y", 0.0);
  pose_to_publish_.pose.pose.position.z = private_nh_.param<double>("pose_z", 0.0);
  double yaw = private_nh_.param<double>("pose_yaw", 0.0);
  // yaw
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  pose_to_publish_.pose.pose.orientation.x = quaternion.x();
  pose_to_publish_.pose.pose.orientation.y = quaternion.y();
  pose_to_publish_.pose.pose.orientation.z = quaternion.z();
  pose_to_publish_.pose.pose.orientation.w = quaternion.w();
  pose_to_publish_.pose.covariance.assign(0.0);
  // covariance (Default value is picked from rviz)
  /*pose_to_publish_.covariance[0] = private_nh_.param<double>("cov_x", 0.25);
  pose_to_publish_.covariance[7] = private_nh_.param<double>("cov_y", 0.25);
  pose_to_publish_.covariance[14] = private_nh_.param<double>("cov_z", 0.0);
  pose_to_publish_.covariance[35] = private_nh_.param<double>("cov_yaw", (M_PI/12.0)*(M_PI/12.0));*/
  pose_to_publish_.pose.covariance[0] = 0.25;                            // cov of x
  pose_to_publish_.pose.covariance[7] = 0.25;                            // cov of y
  pose_to_publish_.pose.covariance[14] = 0.0;                            // cov of z
  pose_to_publish_.pose.covariance[35] = (M_PI / 12.0) * (M_PI / 12.0);  // cov of yaw

  // Advertise
  initial_pose_publisher_ = private_nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);

  // Check params
  bool has_valid_param = private_nh_.hasParam("pose_x") && private_nh_.hasParam("pose_y") &&
                         private_nh_.hasParam("pose_z") && private_nh_.hasParam("pose_yaw");

  // Publish once
  if (has_valid_param)
  {
    initial_pose_publisher_.publish(pose_to_publish_);
  }
  else
  {
    // Won't publish initialpose if pose is not set.
    ROS_WARN("[initial_pose_publisher] One or more pose parameters are not set. Won't publish initialpose.");
  }
}

void InitialPosePublisher::run()
{
  ros::spin();
}

}  // namespace initial_pose_publisher
