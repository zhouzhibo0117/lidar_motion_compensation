
#include <lidar_motion_compensation/lidar_frame.h>
#include <lidar_motion_compensation/callback.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "motionCompensation");
    ros::NodeHandle nh("~");
    nh.param<double>("pose_to_base_x", pose_to_base_x, 0.0);
    nh.param<double>("pose_to_base_y", pose_to_base_y, 0.0);
    nh.param<double>("pose_to_base_z", pose_to_base_z, 0.0);
    nh.param<double>("pose_to_base_yaw", pose_to_base_yaw, 0.0);
    nh.param<double>("pose_to_base_pitch", pose_to_base_pitch, 0.0);
    nh.param<double>("pose_to_base_roll", pose_to_base_roll, 0.0);
    nh.param<double>("velocity_x", velocity_x, 0.0);
    nh.param<double>("velocity_y", velocity_y, 0.0);
    nh.param<double>("velocity_z", velocity_z, 0.0);
    nh.param<double>("velocity_yaw", velocity_yaw, 0.0);
    nh.param<double>("velocity_pitch", velocity_pitch, 0.0);
    nh.param<double>("velocity_roll", velocity_roll, 0.0);
    nh.param<bool>("LIVOX_MID40_1", LIVOX_MID40_1, false);
    nh.param<bool>("LIVOX_MID40_2", LIVOX_MID40_2, false);
    nh.param<bool>("VELODYNE_VLP16", VELODYNE_VLP16, false);
    nh.param<bool>("PANDAR_40", PANDAR_40, false);
    nh.param<double>("LIVOX_FREQUENCY", LIVOX_FREQUENCY, 20.0);
    nh.param<double>("VELODYNE_FREQUENCY", VELODYNE_FREQUENCY, 10.0);
    nh.param<double>("HESAI_FREQUENCY", HESAI_FREQUENCY, 10.0);

    nh.getParam("pose_to_base_x", pose_to_base_x);
    nh.getParam("pose_to_base_y", pose_to_base_y);
    nh.getParam("pose_to_base_z", pose_to_base_z);
    nh.getParam("pose_to_base_yaw", pose_to_base_yaw);
    nh.getParam("pose_to_base_pitch", pose_to_base_pitch);
    nh.getParam("pose_to_base_roll", pose_to_base_roll);
    nh.getParam("velocity_x", velocity_x);
    nh.getParam("velocity_y", velocity_y);
    nh.getParam("velocity_z", velocity_z);
    nh.getParam("velocity_yaw", velocity_yaw);
    nh.getParam("velocity_pitch", velocity_pitch);
    nh.getParam("velocity_roll", velocity_roll);
    nh.getParam("LIVOX_MID40_1", LIVOX_MID40_1);
    nh.getParam("LIVOX_MID40_2", LIVOX_MID40_2);
    nh.getParam("VELODYNE_VLP16", VELODYNE_VLP16);
    nh.getParam("PANDAR_40", PANDAR_40);
    nh.getParam("LIVOX_FREQUENCY", LIVOX_FREQUENCY);
    nh.getParam("VELODYNE_FREQUENCY", VELODYNE_FREQUENCY);
    nh.getParam("HESAI_FREQUENCY", HESAI_FREQUENCY);

    pub_pointcloud1 = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_after", 10);
    pub_pointcloud2 = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_after2", 10);
    pub_pointcloud3 = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_after", 10);
    pub_pointcloud4 = nh.advertise<sensor_msgs::PointCloud2>("/pandar_points_after", 10);

    ros::Subscriber sub1, sub2, sub3, sub4;
    sub1 = nh.subscribe("/livox/lidar", 5, HandlePointCloud1);
    sub2 = nh.subscribe("/livox/lidar2", 5, HandlePointCloud2);
    sub3 = nh.subscribe("/velodyne_points", 5, HandlePointCloud3);
    sub4 = nh.subscribe("/pandar_points", 5, HandlePointCloud4);
    ros::spin();
    return 0;
}