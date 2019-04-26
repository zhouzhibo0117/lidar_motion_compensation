//
// Created by zhibo on 4/26/19.
//

#ifndef SRC_LIDAR_FRAME_H
#define SRC_LIDAR_FRAME_H

#include <base.h>

class LidarFrame {
private:
    const sensor_msgs::PointCloud2ConstPtr &lidar_frame_ptr_;
    const unsigned int lidar_frame_size_;
    const float lidar_frame_hz_;
    const float frame_interval_;
    const float point_interval_;
    SixDofPose base_pose_;
    SixDofVelocity lidar_frame_velocity_;
    sensor_msgs::PointCloud2 pointcloud_compensated;

public:
    LidarFrame(const sensor_msgs::PointCloud2ConstPtr &msg,
               float frequency,
               SixDofPose base_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
               SixDofVelocity velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) : lidar_frame_ptr_(msg),
                                                                           lidar_frame_size_(msg->width),
                                                                           lidar_frame_hz_(frequency),
                                                                           frame_interval_(1 / frequency),
                                                                           point_interval_(
                                                                                   1 / (frequency * msg->width + 0.01)),
                                                                           base_pose_(base_pose),
                                                                           lidar_frame_velocity_(velocity) {}

private:
    void CompensateLidarMotion();

public:
    void PublishCompensatedPointCloud();

    void Display();
};

#endif //SRC_LIDAR_FRAME_H
