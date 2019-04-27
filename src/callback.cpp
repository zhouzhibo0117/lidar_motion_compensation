//
// Created by zhibo on 4/26/19.
//

#include <lidar_motion_compensation/callback.h>

//extern ros::Publisher pub_pointcloud;

void HandlePointCloud(const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg){
    LidarFrame frame(pointcloud_msg,20.0);
    frame.PublishCompensatedPointCloud(pub_pointcloud);
//    sensor_msgs::PointCloud2 pointCloud_to_publish=frame.GetPointCloud2Msgs();
//    pub_pointcloud.publish(pointCloud_to_publish);
//    std::cout<<a<<'\n';

    frame.Display();
}