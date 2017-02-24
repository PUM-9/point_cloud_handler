//
// Created by hampus on 2017-02-21.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <vector>
#include "ros/ros.h"
#include "point_cloud_handler/GetPointCloud.h"
#include "treedwrapper/WrapperScan.h"

typedef sensor_msgs::PointCloud2 pointCloudMessage;
typedef pcl::PointCloud<pcl::PointXYZ> pointCloud;
typedef treedwrapper::WrapperScan scanService;
typedef point_cloud_handler::GetPointCloud getPointCloud;
typedef int degrees;
typedef unsigned short int uint16;

struct scanData 
{
    pointCloudMessage point_cloud_message;
    degrees y_angle;
    degrees x_angle;
};

void
generate_scans(uint16 accuracy, std::vector<scanData>& scans) 
{

    degrees y_rotate = 100 / accuracy;
    degrees x_rotate = 90 / accuracy;
    degrees y_min = 0;
    degrees y_max = 359;
    degrees x_min = -20;
    degrees x_max = 90;
    double times_to_scan_x = std::floor((x_max - x_min) / x_rotate);
    double times_to_scan_y = std::floor((y_max - y_min) / y_rotate);
    degrees y_angle = y_min;
    degrees x_angle;
    ros::NodeHandle node_handle;
    ros::ServiceClient client = node_handle.serviceClient<scanService>("wrapper_scan");

    for (int i = 0; i < times_to_scan_y; ++i)
    {
        x_angle = x_min;
        for (int j = 0; j < times_to_scan_x; ++j)
        {
            x_angle = x_angle + x_rotate;
            scanService srv;
            srv.request.y_angle = y_angle;
            srv.request.x_angle = x_angle;
            if (client.call(srv) && !srv.response.exit_code)
            {
                scanData scan_data;
                scan_data.y_angle = y_angle;
                scan_data.x_angle = x_angle;
                scan_data.point_cloud_message = srv.response.point_cloud;
                scans.push_back(scan_data);
            } else 
            {

            }
        }
        y_angle = y_angle + y_rotate;
    }
}

bool 
get_point_cloud(getPointCloud::Request &req, getPointCloud::Response &resp)
{
    std::vector<scanData> scans;
    generate_scans(req.accuracy, scans);
    
    resp.point_cloud = scans[0].point_cloud_message;
    resp.exit_code = 0;
    resp.error_message = "hej";

    return true;
}    

int
main (int argc, char** argv)
{
    
    ros::init(argc, argv, "point_cloud_handler");
    ros::NodeHandle node_handle;
    ros::ServiceServer service = node_handle.advertiseService("get_point_cloud", get_point_cloud);
    ROS_INFO("Ready to serve point clouds");
    ros::spin();

    return (0);
}
