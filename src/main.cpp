#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <vector>
#include "ros/ros.h"
#include "point_cloud_handler/GetPointCloud.h"
#include "treedwrapper/WrapperScan.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <string>

typedef sensor_msgs::PointCloud2 PointCloudMessage;
typedef treedwrapper::WrapperScan ScanService;
typedef point_cloud_handler::GetPointCloud GetPointCloud;
typedef int degrees;
typedef unsigned short int uint16;


typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef Cloud::Ptr CloudPtr;

/**
    Struct that will be used to store the point clouds as an object.
*/
struct scanData 
{
    PointCloudMessage point_cloud_message;
    degrees y_angle;
    degrees x_angle;
};

float find_depth(CloudPtr cloud) {

    float biggest_z = cloud->points[0].z;
    float smallest_z = biggest_z;

    for (int i=1; i < cloud->size(); i++) {

        float current_z = cloud->points[i].z;

        if (current_z > biggest_z) {
            biggest_z = current_z;
        }

        if (current_z < smallest_z) {
            smallest_z = current_z;
        }

    }

    return biggest_z - smallest_z;

}


void filter(const CloudPtr before, CloudPtr after) {
    CloudPtr temp_cloud_ptr = before;
    pcl::PassThrough<Point> pass;


    // Filter out stick
    pass.setInputCloud(temp_cloud_ptr);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(400, 513);
    pass.setFilterLimitsNegative(false);
    pass.filter(*after);

    temp_cloud_ptr = after;

    pass.setInputCloud(temp_cloud_ptr);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-1, 1);
    pass.setFilterLimitsNegative(true);
    pass.filter(*after);

    temp_cloud_ptr = after;

    // Remove points that are far away from one another.
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
    outlier_filter.setInputCloud(temp_cloud_ptr);
    outlier_filter.setRadiusSearch(0.8);
    outlier_filter.setMinNeighborsInRadius(2);
    outlier_filter.filter(*after);



}

void
message_to_cloud(const PointCloudMessage &message, CloudPtr cloud) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(message,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
}

degrees
get_start_angle(ros::ServiceClient client)
{

    CloudPtr current_cloud_ptr = nullptr;
    CloudPtr optimal_cloud_ptr = nullptr;
    CloudPtr temp_cloud_ptr = nullptr;
    degrees optimal_angle;
    float optimal_depth;
    const unsigned int times_to_scan = 5;
    degrees rotation = 90/times_to_scan;


    // Setup the scan service and request the service to set the angles.
    ScanService srv;
    srv.request.x_angle = 0;
    // Call the ScanService (wrapper), it will return true if service call succeeded, it will return false if the call not succeed.
    // It will also check if the exit_code is valid (return 0).

    for (degrees v=0; v <= 90; v+=rotation) {

        srv.request.y_angle = v;

        if (client.call(srv) && !srv.response.exit_code) {

            const PointCloudMessage msg = srv.response.point_cloud;

            message_to_cloud(msg, temp_cloud_ptr);

            filter(temp_cloud_ptr, current_cloud_ptr);

            float current_depth = find_depth(current_cloud_ptr);

            if (!optimal_cloud_ptr || current_depth < optimal_depth) {
                optimal_cloud_ptr = current_cloud_ptr;
                optimal_depth = current_depth;
                optimal_angle = v;
            }
        }
    }
    return optimal_angle;
}

/**
    This function will call the treed_wrapper node to generate scans.
    Every scan will be stored as an scanData object which consist of angles of the board and the point cloud. 
    @param accuracy This will set the precision on the scan, in other words, how many times to scan.
    @param scans The vector to store all scanData objects in.
    @return true if success, false otherwise.
*/
bool
generate_scans(uint16 accuracy, std::vector<scanData> &scans)
{
    // Constant number for the degrees of the rotation board.
    degrees y_min = 0;
    degrees y_max = 359;
    degrees x_min = -20;
    degrees x_max = 90;
    
    // Define how many degrees to rotate every time.
    degrees y_rotate = 100 / accuracy;
    degrees x_rotate = 90 / accuracy;

    // Define how many times to scan in both y and x directions.
    double times_to_scan_x = std::floor((x_max - x_min) / x_rotate);
    double times_to_scan_y = std::floor((y_max - y_min) / y_rotate);
    
    // Define the angles for y and x axis.
    degrees y_angle;
    degrees x_angle = x_min;
    
    // Define the client that will be used to call the wrapper_scan service. 
    ros::NodeHandle node_handle;
    ros::ServiceClient client = node_handle.serviceClient<ScanService>("wrapper_scan");
    
    // For every angle in x, rotate the board on every angle on the y axis.    
    for (int i = 0; i < times_to_scan_x; ++i) {
        y_angle = y_min;
        for (int j = 0; j < times_to_scan_y; ++j) {
            y_angle = y_angle + y_rotate;
            
            // Setup the scan service and request the service to set the angles.
            ScanService srv;
            srv.request.y_angle = y_angle;
            srv.request.x_angle = x_angle;
            
            // Call the ScanService (wrapper), it will return true if service call succeeded, it will return false if the call not succeed.
            // It will also check if the exit_code is valid (return 0).
            if (client.call(srv) && !srv.response.exit_code) {
                scanData scan_data;
                scan_data.y_angle = y_angle;
                scan_data.x_angle = x_angle;
                scan_data.point_cloud_message = srv.response.point_cloud;
                scans.push_back(scan_data);
            } else {
                return false;
            }
        }
        x_angle = x_angle + x_rotate;
    }
    return true;
}

/**
    This function will scan an object and register all the point clouds to a resulting point cloud.
    @param req The service request that has been made.
    @param resp The response, vector which contains the scanned point clouds.
    @return true when the process to gathering and regiser point clouds is done.
*/
bool 
get_point_cloud(GetPointCloud::Request &req, GetPointCloud::Response &resp)
{

    ros::NodeHandle node_handle;
    ros::ServiceClient client = node_handle.serviceClient<ScanService>("wrapper_scan");

    degrees start = get_start_angle(client);
    std::cout << "start at angle: " << start << std::endl;
    // Vector with scanData objects. scanData objects consist of one point cloud and the angles for the rotation board.
    std::vector<scanData> scans;
    ScanService srv;
    degrees x_angle = 0;
    degrees y_angle = 0;
    srv.request.x_angle = x_angle;

    for (int i=0; i < 4; i++) {
        y_angle = start + i*90;
        srv.request.y_angle = y_angle;
        if (client.call(srv) && !srv.response.exit_code) {
            scanData scan_data;
            CloudPtr cloud_to_save_ptr;
            scan_data.y_angle = y_angle;
            scan_data.x_angle = x_angle;
            scan_data.point_cloud_message = srv.response.point_cloud;
            scans.push_back(scan_data);
            message_to_cloud(srv.response.point_cloud, cloud_to_save_ptr);
            pcl::io::savePCDFile("scan_" + std::to_string(i),*cloud_to_save_ptr);

        } else {
            return false;
        }
    }

    return true;
}


/**
    Main function that will init a ros node and spin up the get_point_cloud service. 
    @param argc Used to parse arguments from the command line.
    @param argv Used to parse arguments from the command line.
    @return None.
*/
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
