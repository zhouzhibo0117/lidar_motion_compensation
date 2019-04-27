//
// Created by zhibo on 4/26/19.
//

#ifndef SRC_CALLBACK_H
#define SRC_CALLBACK_H

#include <base.h>
#include <lidar_frame.h>

void HandlePointCloud1(const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg);

void HandlePointCloud2(const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg);

#endif //SRC_CALLBACK_H
