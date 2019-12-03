initial_pose_publisher
=============================================
The node that publish initial pose by rosparam when it launched.


Publishing topics
-----------------------

| Topic | Type | Description |
|------|------|-------------|
| /initialpose | geometry_msgs::PoseWithCovarianceStamped | Initial pose |

Parameters
-----------------------

| Name | Type | Description | Default value |
|------|------|-------------|---------------|
| ~pose_x, ~pose_y, ~pose_z, ~pose_yaw | float64 | initial pose in specified frame | 0.0 |
| ~frame_id | string | frame id | base_link |