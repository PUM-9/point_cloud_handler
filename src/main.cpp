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
//#include "treedwrapper/WrapperScan"

typedef sensor_msgs::PointCloud2 PointCloudMessage;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef float degrees;
typedef unsigned short int uint16;

void
scan_object(uint16 accuracy) {

    degrees y_rotate = 100 / accuracy;
    degrees x_rotate = 90 / accuracy;
    degrees y_min = 0;
    degrees y_max = 359;
    degrees x_min = -20;
    degrees x_max = 90;
    int times_to_scan_x = std::floor((x_max - x_min) / x_rotate);
    int times_to_scan_y = std::floor((y_max - y_min) / y_rotate);
    degrees y_angle = y_min;
    degrees x_angle;
    //std::vector<PointCloud> point_clouds = std::vector<PointCloud>();

    for (int i = 0; i < times_to_scan_y; ++i)
    {
        x_angle = x_min;
        std::cout << "y: " << y_angle << ", ";
        for (int j = 0; j < times_to_scan_x; ++j)
        {
            std::cout << "x: " << x_angle << ", ";
            x_angle = x_angle + x_rotate;
        }
        y_angle = y_angle + y_rotate;
    }

    return;

}


int
main (int argc, char** argv)
{
    std::cout << std::endl << "accuracy 1:" << std::endl;
    scan_object(1);
    std::cout << std::endl << "accuracy 3:" << std::endl;
    scan_object(3);
    std::cout << std::endl << "accuracy 7:" << std::endl;
    scan_object(7);
    return (0);
}