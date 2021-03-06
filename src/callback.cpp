//
// Created by zhibo on 4/26/19.
//

#include <lidar_motion_compensation/callback.h>

ros::Publisher pub_pointcloud1;
ros::Publisher pub_pointcloud2;
ros::Publisher pub_pointcloud3;
ros::Publisher pub_pointcloud4;

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

double LIVOX_FREQUENCY;
double VELODYNE_FREQUENCY;
double HESAI_FREQUENCY;

bool VELODYNE_VLP16;
bool LIVOX_MID40_1;
bool LIVOX_MID40_2;
bool PANDAR_40;

void HandlePointCloud1(const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg) {
    if (LIVOX_MID40_1) {
        LidarFrame frame(pointcloud_msg, LIVOX_FREQUENCY);
        frame.PublishCompensatedPointCloud(pub_pointcloud1);
    }
}

void HandlePointCloud2(const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg) {
    if (LIVOX_MID40_2) {
        LidarFrame frame(pointcloud_msg, LIVOX_FREQUENCY);
        frame.SetTransformationMatrix(pose_to_base_x, pose_to_base_y, pose_to_base_z,
                                      pose_to_base_roll, pose_to_base_pitch, pose_to_base_yaw,
                                      velocity_x, velocity_y, velocity_z,
                                      velocity_roll, velocity_pitch, velocity_yaw);
        frame.PublishCompensatedPointCloud(pub_pointcloud2);
    }
}

void HandlePointCloud3(const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg) {
    if (VELODYNE_VLP16) {
        LidarFrame frame(pointcloud_msg, VELODYNE_FREQUENCY);
        frame.SetTransformationMatrix(pose_to_base_x, pose_to_base_y, pose_to_base_z,
                                      pose_to_base_roll, pose_to_base_pitch, pose_to_base_yaw,
                                      velocity_x, velocity_y, velocity_z,
                                      velocity_roll, velocity_pitch, velocity_yaw);
        frame.PublishCompensatedPointCloud(pub_pointcloud3);
    }
}

void HandlePointCloud4(const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg) {
    if (PANDAR_40) {
        LidarFrame frame(pointcloud_msg, HESAI_FREQUENCY);
        frame.SetTransformationMatrix(pose_to_base_x, pose_to_base_y, pose_to_base_z,
                                      pose_to_base_roll, pose_to_base_pitch, pose_to_base_yaw,
                                      velocity_x, velocity_y, velocity_z,
                                      velocity_roll, velocity_pitch, velocity_yaw);
        frame.PublishCompensatedPointCloud(pub_pointcloud4);
    }
}