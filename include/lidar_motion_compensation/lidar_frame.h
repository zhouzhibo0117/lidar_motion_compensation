//
// Created by zhibo on 4/26/19.
//

#ifndef SRC_LIDAR_FRAME_H
#define SRC_LIDAR_FRAME_H

#include <base.h>

class LidarFrame {
private:
    const sensor_msgs::PointCloud2ConstPtr &lidar_frame_ptr_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_frame_before_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_frame_after_;
    const unsigned int lidar_frame_size_;
    unsigned int lidar_frame_after_size_;
    const float lidar_frame_hz_;
    const float frame_interval_;
    const float point_interval_;
    SixDofPose pose_to_base_;
    SixDofVelocity lidar_frame_velocity_;
    double frame_time;

public:
    LidarFrame(const sensor_msgs::PointCloud2ConstPtr &msg, float frequency);


private:
    void CalcVelocity();

    void CompensateLidarMotion();

public:
    void SetTransformationMatrix(float translation_x, float translation_y, float translation_z,
                                 float rotation_roll, float rotation_pitch, float rotation_yaw,
                                 float velocity_x, float velocity_y, float velocity_z,
                                 float velocity_roll, float velocity_pitch, float velocity_yaw);

    void PublishCompensatedPointCloud(ros::Publisher &publisher);

};

#endif //SRC_LIDAR_FRAME_H
