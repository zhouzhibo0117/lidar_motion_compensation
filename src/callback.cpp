//
// Created by zhibo on 4/26/19.
//

#include <lidar_motion_compensation/callback.h>

void HandlePointCloud(const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg){
    LidarFrame frame(pointcloud_msg,20.0);
    frame.Display();
}