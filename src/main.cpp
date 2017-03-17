#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <vector>
#include "ros/ros.h"
#include "point_cloud_handler/GetPointCloud.h"
#include "treedwrapper/WrapperScan.h"

typedef sensor_msgs::PointCloud2 PointCloudMessage;
typedef treedwrapper::WrapperScan ScanService;
typedef point_cloud_handler::GetPointCloud GetPointCloud;
typedef int degrees;
typedef unsigned short int uint16;

/**
    Struct that will be used to store the point clouds as an object.
*/
struct scanData 
{
    PointCloudMessage point_cloud_message;
    degrees y_angle;
    degrees x_angle;
};

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
    // Check if the accuracy is valid.
    if (req.accuracy <= 0 || req.accuracy > 10) {
        resp.exit_code = 1;
        resp.error_message = "Invalid accuracy, should be between 1 and 10";
        return true;
    }
    
    // Vector with scanData objects. scanData objects consist of one point cloud and the angles for the rotation board.
    std::vector<scanData> scans;
    
    // Generate the scans, it will return true if success to call the wrapper service, false if not not succeed.
    if (generate_scans(req.accuracy, scans)) {
        // The code for point cloud registration should be implemented here, the sample below is just test code.
        resp.point_cloud = scans[0].point_cloud_message;
        resp.exit_code = 0;
        resp.error_message = "";    
    } else {
        resp.exit_code = 1;
        resp.error_message = "Failed to call service wrapper_scan";
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
