initial_pose_publisher
=============================================

The node that publish initial pose by rosparam when it launched.

Description
-----------------------

* The node publishes initialpose when it launched if all parameters are set.
* If one or more pose parameters are lacking, it won't publish.
* The node waits to publish initialpose atleast one node subscribes it.


Publishing topics
-----------------------

| Topic | Type | Description |
|------|------|-------------|
| /initialpose | geometry_msgs::PoseWithCovarianceStamped | Initial pose |

Parameters
-----------------------

| Name | Type | Description | Default value |
|------|------|-------------|---------------|
| ~pose_x, ~pose_y, ~pose_z, ~pose_yaw | float64 | initial pose in specified frame (Must specify) | - |
| ~frame_id | string | frame id | base_link |