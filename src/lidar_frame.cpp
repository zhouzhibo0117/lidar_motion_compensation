//
// Created by zhibo on 4/26/19.
//

#include "lidar_motion_compensation/lidar_frame.h"

//LidarFrame::LidarFrame(/*TODO*/) {}

void LidarFrame::CompensateLidarMotion() {
    float point_time = 0.0;
    for (int i = 0; i < lidar_frame_size_; i++) {
        //TODO: Transform point.
        //TODO: Write to new pointcloud.

        point_time += point_interval_;
    }
}


void LidarFrame::PublishCompensatedPointCloud(){
    LidarFrame::CompensateLidarMotion();

    //TODO: Publish pointcloud.
}

void LidarFrame::Display() {
    ROS_INFO("Frame size: %d", lidar_frame_size_);
}
