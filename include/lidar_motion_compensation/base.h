//
// Created by zhibo on 4/26/19.
//

#ifndef SRC_BASE_H
#define SRC_BASE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

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

#endif //SRC_BASE_H