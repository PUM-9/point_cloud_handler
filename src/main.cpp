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
#include <sstream>

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

typedef sensor_msgs::PointCloud2 PointCloudMessage;
typedef treedwrapper::WrapperScan ScanService;
typedef point_cloud_handler::GetPointCloud GetPointCloud;
typedef int degrees;
typedef unsigned short int uint16;
typedef float millimeter;


typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef Cloud::Ptr CloudPtr;

/**
    Struct that will be used to store the point clouds as an object.
*/


struct Cuboid {
    CloudPtr point_cloud_ptr;
    degrees pov_angle;
    Point origo, x, y, z, xy, xz, yz, xyz;
};

struct Rectangle {
    CloudPtr point_cloud_ptr;
    degrees pov_angle;
    Point origo, x, y, xy;
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

bool is_rectangle(const Rectangle &rectangle) {


    float origo_to_x = std::abs(rectangle.x.x - rectangle.origo.x);
    float x_to_xy = std::abs(rectangle.xy.y - rectangle.x.y);
    float y_to_xy = std::abs(rectangle.xy.x - rectangle.y.x);
    float origo_to_y = std::abs(rectangle.y.y - rectangle.origo.y);

    const int tolerance_factor = 20;
    float x_tolerance = origo_to_x / tolerance_factor;
    float y_tolerance = origo_to_y / tolerance_factor;

    return (std::abs(origo_to_x - y_to_xy) < x_tolerance) && (std::abs(origo_to_y - x_to_xy) < y_tolerance);

}

bool closer_rect_origo(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x + current_point.y;
    float new_sum = new_point.x + new_point.y;
    return new_sum < current_sum;
}

bool closer_rect_x(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x - current_point.y;
    float new_sum = new_point.x - new_point.y;
    return new_sum > current_sum;
}

bool closer_rect_y(const Point current_point, const Point &new_point) {
    float current_sum = current_point.y - current_point.x;
    float new_sum = new_point.y - new_point.x;
    return new_sum > current_sum;
}

bool closer_rect_xy(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x + current_point.y;
    float new_sum = new_point.x + new_point.y;
    return new_sum > current_sum;
}

void set_rectangle_corners(Rectangle &rectangle) {

    Cloud point_cloud = *rectangle.point_cloud_ptr;
    rectangle.origo = point_cloud.points[0];
    rectangle.x = point_cloud.points[0];
    rectangle.y = point_cloud.points[0];
    rectangle.xy = point_cloud.points[0];

    for (int i = 1; i < point_cloud.size(); i++){

        Point current_point = point_cloud.points[i];

        if (closer_rect_origo(rectangle.origo, current_point)) {
            rectangle.origo = current_point;
        }

        if (closer_rect_x(rectangle.x, current_point)) {
            rectangle.x = current_point;
        }

        if (closer_rect_y(rectangle.y, current_point)) {
            rectangle.y = current_point;
        }

        if (closer_rect_xy(rectangle.xy, current_point)) {
            rectangle.xy = current_point;
        }
    }
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
get_start_angle(ros::ServiceClient client, const unsigned int times_to_scan)
{
    CloudPtr current_cloud_ptr (new Cloud());
    CloudPtr optimal_cloud_ptr (new Cloud());
    CloudPtr temp_cloud_ptr (new Cloud());
    degrees optimal_angle;
    float optimal_depth;
    degrees rotation = 90/times_to_scan;
    bool first = true;

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

            if (first || current_depth < optimal_depth) {
                optimal_cloud_ptr = current_cloud_ptr;
                optimal_depth = current_depth;
                optimal_angle = v;
                first = false;
            }
        }
    }
    return optimal_angle;
}

std::vector<Rectangle> scan(ros::ServiceClient client, const unsigned int accuracy) {
    degrees start = get_start_angle(client, accuracy);
    std::cout << "start at angle: " << start << std::endl;
    // Vector with scanData objects. scanData objects consist of one point cloud and the angles for the rotation board.
    std::vector<Rectangle> scans;
    ScanService srv;
    degrees x_angle = 0;
    degrees y_angle = 0;
    srv.request.x_angle = x_angle;
    for (int i=0; i < 4; i++) {
        y_angle = start + i*90;
        srv.request.y_angle = y_angle;
        if (client.call(srv) && !srv.response.exit_code) {
            Rectangle scan;
            CloudPtr cloud_to_save_ptr (new Cloud());
            CloudPtr cloud_temp_ptr (new Cloud());
            scan.pov_angle = y_angle;
            message_to_cloud(srv.response.point_cloud, cloud_temp_ptr);
            filter(cloud_temp_ptr, cloud_to_save_ptr);
            scan.point_cloud_ptr = cloud_to_save_ptr;
            set_rectangle_corners(scan);
            if (!is_rectangle(scan)) {
                throw 2; // It's no rectangle
            }
            scans.push_back(scan);

        } else {
            throw 1; // request to treedwrapper failed.
        }
    }
    return scans;
}

Cuboid generate_cuboid(const millimeter width, const millimeter height, const millimeter depth) {

    millimeter step_size = 0.1;
    CloudPtr point_cloud_ptr (new Cloud());
    Cuboid cuboid;

    // Create two xy planes in the xzy space.
    for (millimeter i=0; i<width; i+=step_size) {

        for (millimeter j=0; j<height; j+=step_size) {
            //Add one point for each side of the cube
            Point point1 = Point(i, j, 0);
            Point point2 = Point(i, j, depth);
            point_cloud_ptr->push_back(point1);
            point_cloud_ptr->push_back(point2);
        }

    }

    // Create two yz planes in the xyz space.
    for (millimeter i=0; i<depth; i+=step_size) {

        for (millimeter j=0; j<height; j+=step_size) {
            Point point1 = Point(0, j, i);
            Point point2 = Point(width, j, i);
            point_cloud_ptr->push_back(point1);
            point_cloud_ptr->push_back(point2);
        }

    }

    // Create two xz planes in the xyz space.
    for (millimeter i=0; i<depth; i+=step_size) {

        for (millimeter j=0; j<width; j+=step_size) {
            Point point1 = Point(j, 0, i);
            Point point2 = Point(j, height, i);
            point_cloud_ptr->push_back(point1);
            point_cloud_ptr->push_back(point2);
        }

    }

    cuboid.point_cloud_ptr = point_cloud_ptr;
    cuboid.pov_angle = 0;
    cuboid.origo = Point(0, 0, 0);
    cuboid.x = Point(width, 0, 0);
    cuboid.y = Point(0, height, 0);
    cuboid.z = Point(0, 0, depth);
    cuboid.xy = Point(width, height, 0);
    cuboid.xz = Point(width, 0, depth);
    cuboid.yz = Point(0, height, depth);
    cuboid.xyz = Point(width, height, depth);

    return cuboid;

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
    std::vector<Rectangle> scans;

    try {
        scans = scan(client, req.accuracy);
    } catch (short e) {
        switch (e) {
            case 1:
                resp.error_message = "Failed to call service wrapper_scan";
                break;
            case 2:
                resp.error_message = "Not a rectangle";
                break;
            default:
                resp.error_message = "Unknown error";
        }
        resp.exit_code = 1;
        return false;
    }

    millimeter width = std::abs(scans[0].x.x - scans[0].origo.x);
    millimeter height = std::abs(scans[0].y.y - scans[0].origo.y);
    millimeter depth = std::abs(scans[1].x.x - scans[1].origo.x);
    Cuboid cuboid = generate_cuboid(width, height, depth);

    for (int i)

    pcl::io::savePCDFile("cuboid.pcd", *cuboid.point_cloud_ptr);

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
