//
// Created by zhibo on 4/26/19.
//

#ifndef SRC_BASE_H
#define SRC_BASE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>


struct SixDofPose{
    float x; // Unit(m)
    float y;
    float z;
    float yaw; // Unit(rad)
    float pitch;
    float roll;
};

struct SixDofVelocity{
    float x; // Unit(m/s)
    float y;
    float z;
    float yaw; // Unit(rad/s)
    float pitch;
    float roll;
};

extern ros::Publisher pub_pointcloud1;
extern ros::Publisher pub_pointcloud2;
extern ros::Publisher pub_pointcloud3;
extern ros::Publisher pub_pointcloud4;

extern double pose_to_base_x;
extern double pose_to_base_y;
extern double pose_to_base_z;
extern double pose_to_base_yaw;
extern double pose_to_base_pitch;
extern double pose_to_base_roll;
extern double velocity_x;
extern double velocity_y;
extern double velocity_z;
extern double velocity_yaw;
extern double velocity_pitch;
extern double velocity_roll;

extern double VELODYNE_FREQUENCY;
extern double LIVOX_FREQUENCY;
extern double HESAI_FREQUENCY;

extern bool VELODYNE_VLP16;
extern bool LIVOX_MID40_1;
extern bool LIVOX_MID40_2;
extern bool PANDAR_40;

#endif //SRC_BASE_H