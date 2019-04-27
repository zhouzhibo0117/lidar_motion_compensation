
#include <lidar_motion_compensation/lidar_frame.h>
#include <lidar_motion_compensation/callback.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"motionCompensation");
    ros::NodeHandle nh;
    pub_pointcloud=nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_after",10);

    ros::Subscriber sub=nh.subscribe("/livox/lidar",5,HandlePointCloud);
    ros::spin();
    return 0;
}