//
// Created by zhibo on 4/26/19.
//

#include <lidar_motion_compensation/callback.h>

ros::Publisher pub_pointcloud1;
ros::Publisher pub_pointcloud2;
double pose_to_base_x;
double pose_to_base_y;
double pose_to_base_z;
double pose_to_base_yaw;
double pose_to_base_pitch;
double pose_to_base_roll;
double velocity_x;
double velocity_y;
double velocity_z;
double velocity_yaw;
double velocity_pitch;
double velocity_roll;


void HandlePointCloud1(const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg) {
    LidarFrame frame1(pointcloud_msg, 20.0);
    frame1.PublishCompensatedPointCloud(pub_pointcloud1);
}

void HandlePointCloud2(const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg) {
    LidarFrame frame2(pointcloud_msg, 20.0);
    frame2.SetTransformationMatrix(pose_to_base_x,pose_to_base_y,pose_to_base_z,
                                   pose_to_base_roll,pose_to_base_pitch,pose_to_base_yaw,
                                   velocity_x,velocity_y,velocity_z,
                                   velocity_roll,velocity_pitch,velocity_yaw);
    frame2.PublishCompensatedPointCloud(pub_pointcloud2);
}