
#include <lidar_motion_compensation/lidar_frame.h>
#include <lidar_motion_compensation/callback.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"motionCompensation");
    ros::NodeHandle nh;

    ros::Subscriber sub=nh.subscribe("/livox/lidar",5,HandlePointCloud);
    ros::spin();
    return 0;
}