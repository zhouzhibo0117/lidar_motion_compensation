//
// Created by zhibo on 4/26/19.
//

#include "lidar_motion_compensation/lidar_frame.h"

//LidarFrame::LidarFrame(/*TODO*/) {}

LidarFrame::LidarFrame(const sensor_msgs::PointCloud2ConstPtr &msg,
                       float frequency) : lidar_frame_ptr_(msg),
                                          lidar_frame_size_(msg->width),
                                          lidar_frame_hz_(frequency),
                                          frame_interval_(1 / frequency),
                                          point_interval_(
                                                  1 / (frequency * msg->width + 0.01)),

                                          lidar_frame_before_(
                                                  new pcl::PointCloud<pcl::PointXYZI>()),
                                          lidar_frame_after_(
                                                  new pcl::PointCloud<pcl::PointXYZI>()) {
    pose_to_base_ = {0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0};
    lidar_frame_velocity_ = {0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0};
    frame_time = lidar_frame_ptr_->header.stamp.toSec();
    lidar_frame_before_->clear();
    pcl::fromROSMsg(*lidar_frame_ptr_, *lidar_frame_before_);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*lidar_frame_before_, *lidar_frame_before_, indices);
}

LidarFrame::LidarFrame(const sensor_msgs::PointCloud2ConstPtr &msg,
                       float frequency,
                       SixDofPose pose_to_base = {0.0, 0.0, 0.0,
                                                  0.0, 0.0, 0.0},
                       SixDofVelocity velocity = {0.0, 0.0, 0.0,
                                                  0.0, 0.0, 0.0}) : lidar_frame_ptr_(msg),
                                                                    lidar_frame_size_(msg->width),
                                                                    lidar_frame_hz_(frequency),
                                                                    frame_interval_(1 / frequency),
                                                                    point_interval_(
                                                                            1 / (frequency * msg->width + 0.01)),
                                                                    pose_to_base_(pose_to_base),
                                                                    lidar_frame_velocity_(velocity),
                                                                    lidar_frame_before_(
                                                                            new pcl::PointCloud<pcl::PointXYZI>()),
                                                                    lidar_frame_after_(
                                                                            new pcl::PointCloud<pcl::PointXYZI>()) {
    frame_time = lidar_frame_ptr_->header.stamp.toSec();
    lidar_frame_before_->clear();
    pcl::fromROSMsg(*lidar_frame_ptr_, *lidar_frame_before_);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*lidar_frame_before_, *lidar_frame_before_, indices);
}

void LidarFrame::CalcVelocity() {

}

void LidarFrame::SetTransformationMatrix(float translation_x, float translation_y, float translation_z,
                                         float rotation_roll, float rotation_pitch, float rotation_yaw,
                                         float velocity_x, float velocity_y, float velocity_z,
                                         float velocity_roll, float velocity_pitch, float velocity_yaw) {
    SixDofPose input_pose = {translation_x, translation_y, translation_z, rotation_yaw, rotation_pitch, rotation_roll};
    SixDofVelocity input_velocity = {velocity_x, velocity_y, velocity_z, velocity_yaw, velocity_pitch, velocity_roll};
    pose_to_base_ = input_pose;
    lidar_frame_velocity_ = input_velocity;
}

void LidarFrame::CompensateLidarMotion() {
    LidarFrame::CalcVelocity();
    SixDofPose delta_pose = {lidar_frame_velocity_.x * point_interval_,
                             lidar_frame_velocity_.y * point_interval_,
                             lidar_frame_velocity_.z * point_interval_,
                             lidar_frame_velocity_.yaw * point_interval_,
                             lidar_frame_velocity_.pitch * point_interval_,
                             lidar_frame_velocity_.roll * point_interval_};
    SixDofPose point_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Matrix3d rotation_matrix_to_base;
    Eigen::Matrix3d inverse_rotation_matrix_to_base;
    Eigen::Vector3d translation_vector_to_base(pose_to_base_.x, pose_to_base_.y, pose_to_base_.z);
    Eigen::Vector3d inverse_translation_vector_to_base(-pose_to_base_.x, -pose_to_base_.y, -pose_to_base_.z);
    rotation_matrix_to_base = Eigen::AngleAxisd(pose_to_base_.yaw, Eigen::Vector3d::UnitZ())
                              * Eigen::AngleAxisd(pose_to_base_.pitch, Eigen::Vector3d::UnitY())
                              * Eigen::AngleAxisd(pose_to_base_.roll, Eigen::Vector3d::UnitX());
    inverse_rotation_matrix_to_base = rotation_matrix_to_base.inverse();


    for (unsigned int i = 0; i < lidar_frame_before_->size(); i++) {
        //TODO: Transform point.
        Eigen::Vector3d point_before(lidar_frame_before_->points[i].x,
                                     lidar_frame_before_->points[i].y,
                                     lidar_frame_before_->points[i].z);
        Eigen::Matrix3d rotation_matrix_moving;
        Eigen::Vector3d translation_vector_moving(point_pose.x, point_pose.y, point_pose.z);
        rotation_matrix_moving = Eigen::AngleAxisd(point_pose.yaw, Eigen::Vector3d::UnitZ())
                                 * Eigen::AngleAxisd(point_pose.pitch, Eigen::Vector3d::UnitY())
                                 * Eigen::AngleAxisd(point_pose.roll, Eigen::Vector3d::UnitX());

        // T_all = T_to_base_inv * T_mov * T_to_base;
        // R_all = R_to_base_inv * R_mov * R_to_base;
        Eigen::Matrix3d R1R2;
        Eigen::Matrix3d rotation_matrix_all;
        R1R2 = inverse_rotation_matrix_to_base * rotation_matrix_moving;
        rotation_matrix_all = R1R2 * rotation_matrix_to_base;
        // t_all = R_to_base_inv * R_mov * t_to_base + R_to_base_inv * t_mov + t_to_base_inv;
        Eigen::Vector3d translation_vector_all;
        translation_vector_all = R1R2 * translation_vector_to_base +
                                 inverse_rotation_matrix_to_base * translation_vector_moving +
                                 inverse_translation_vector_to_base;
        // Rotate.
        Eigen::Vector3d point_after = rotation_matrix_all * point_before;

        pcl::PointXYZI point;
        point.x = point_after(0) + translation_vector_all(0);
        point.y = point_after(1) + translation_vector_all(1);
        point.z = point_after(2) + translation_vector_all(2);
        point.intensity = lidar_frame_before_->points[i].intensity;

        //TODO: Write to new pointcloud.
        lidar_frame_after_->push_back(point);

        point_pose.x += delta_pose.x;
        point_pose.y += delta_pose.y;
        point_pose.z += delta_pose.z;
        point_pose.yaw += delta_pose.yaw;
        point_pose.pitch += delta_pose.pitch;
        point_pose.roll += delta_pose.roll;
    }
}


void LidarFrame::PublishCompensatedPointCloud(ros::Publisher &publisher) {
    LidarFrame::CompensateLidarMotion();

    //TODO: Publish pointcloud.
    sensor_msgs::PointCloud2 compensated_pointcloud;
    pcl::toROSMsg(*lidar_frame_after_, compensated_pointcloud);
    compensated_pointcloud.header.stamp = ros::Time().fromSec(frame_time);
    publisher.publish(compensated_pointcloud);
    lidar_frame_after_size_=compensated_pointcloud.width;

}

//sensor_msgs::PointCloud2 LidarFrame::GetPointCloud2Msgs() {
//    LidarFrame::CompensateLidarMotion();
//
//    pcl::toROSMsg(*lidar_frame_after_, compensated_pointcloud);
//    compensated_pointcloud.header.stamp = ros::Time().fromSec(frame_time);
//    lidar_frame_after_size_=compensated_pointcloud.width;
//    return compensated_pointcloud;
//}

void LidarFrame::Display() {
//    ROS_INFO("[Frame size] before: %d after: %d", lidar_frame_size_,lidar_frame_after_size_);
}
