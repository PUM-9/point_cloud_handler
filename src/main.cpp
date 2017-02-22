//
// Created by hampus on 2017-02-21.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include "ros/ros.h"
#include "point_cloud_handler/GetPointCloud.h"

typedef sensor_msgs::PointCloud2 PointCloud;

bool
get_point_cloud (point_cloud_handler::GetPointCloud::Request &req,
                     point_cloud_handler::GetPointCloud::Response &resp)
{
    PointCloud::Ptr pc (new PointCloud);
    pc->header.frame_id = "some_tf_frame";
    pc->height = pc->width = req.accuracy;
    resp.point_cloud = *pc;
    resp.exit_code = 0;
    resp.error_message = "";
    return true;
}

int
main (int argc, char** argv)
{
    ros::init(argc, argv, "poin_cloud_handler");
    ros::NodeHandle node_handle;

    ros::ServiceServer service = node_handle.advertiseService("get_point_cloud", get_point_cloud);
    ROS_INFO("Ready to serve point clouds");
    ros::spin();

    return (0);
}