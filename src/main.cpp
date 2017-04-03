#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <vector>
#include "ros/ros.h"
#include "point_cloud_handler/GetPointCloud.h"
#include "point_cloud_handler/TestScan.h"
#include "treedwrapper/WrapperScan.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <sstream>
#include <exception>
#include <stdlib.h>

typedef sensor_msgs::PointCloud2 PointCloudMessage;
typedef treedwrapper::WrapperScan ScanService;
typedef point_cloud_handler::GetPointCloud GetPointCloud;
typedef point_cloud_handler::TestScan TestScan;
typedef int degrees;
typedef unsigned short int uint16;
typedef float millimeter;


typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef Cloud::Ptr CloudPtr;

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> CloudI;
typedef CloudI::Ptr CloudIPtr;


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
    Point origo, x, z, xz;
};



float find_depth(CloudPtr cloud) {

    float biggest_y = cloud->points[0].y;
    float smallest_y = biggest_y;

    for (int i=1; i < cloud->size(); i++) {

        float current_y = cloud->points[i].y;

        if (current_y > biggest_y) {
            biggest_y = current_y;
        }

        if (current_y < smallest_y) {
            smallest_y = current_y;
        }

    }

    return biggest_y - smallest_y;

}

bool is_rectangle(const Rectangle &rectangle) {


    float origo_to_x = std::abs(rectangle.x.x - rectangle.origo.x);
    float x_to_xz = std::abs(rectangle.xz.z - rectangle.x.z);
    float z_to_xz = std::abs(rectangle.xz.x - rectangle.z.x);
    float origo_to_z = std::abs(rectangle.z.z - rectangle.origo.z);

    const int tolerance_factor = 10;
    float x_tolerance = origo_to_x / tolerance_factor;
    float z_tolerance = origo_to_z / tolerance_factor;

    std::cout << "origo->x: " << origo_to_x << " x->xz: " << x_to_xz << " z->xz: " << z_to_xz << " o->z: " << origo_to_z << std::endl;
    std::cout << "x tol: " << x_tolerance << " z tol: " << z_tolerance << std::endl;

    return (std::abs(origo_to_x - z_to_xz) < x_tolerance) && (std::abs(origo_to_z - x_to_xz) < z_tolerance);

}

bool closer_cuboid_origo(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x + current_point.y + current_point.z;
    float new_sum = new_point.x + new_point.y + new_point.z;
    return new_sum < current_sum;
}

bool closer_cuboid_x(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x - current_point.y - current_point.z;
    float new_sum = new_point.x - new_point.y - new_point.z;
    return new_sum > current_sum;
}

bool closer_cuboid_y(const Point current_point, const Point &new_point) {
    float current_sum = current_point.y - current_point.x - current_point.z;
    float new_sum = new_point.y - new_point.x - new_point.z;
    return new_sum > current_sum;
}

bool closer_cuboid_z(const Point current_point, const Point &new_point) {
    float current_sum = current_point.z - current_point.y - current_point.x;
    float new_sum = new_point.z - new_point.y - new_point.x;
    return new_sum > current_sum;
}

bool closer_cuboid_xyz(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x + current_point.y + current_point.z;
    float new_sum = new_point.x + new_point.y + new_point.z;
    return new_sum > current_sum;
}

bool closer_cuboid_yz(const Point current_point, const Point &new_point) {
    float current_sum = current_point.y + current_point.z - current_point.x;
    float new_sum = new_point.y + new_point.z - new_point.x;
    return new_sum > current_sum;
}

bool closer_cuboid_xy(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x + current_point.y - current_point.z;
    float new_sum = new_point.x + new_point.y - new_point.z;
    return new_sum > current_sum;
}

bool closer_cuboid_xz(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x - current_point.y + current_point.z;
    float new_sum = new_point.x - new_point.y + new_point.z;
    return new_sum > current_sum;
}

void set_cuboid_corners(Cuboid cuboid) {
    cuboid.origo = cuboid.point_cloud_ptr->points[0];
    cuboid.x = cuboid.point_cloud_ptr->points[0];
    cuboid.y = cuboid.point_cloud_ptr->points[0];
    cuboid.z = cuboid.point_cloud_ptr->points[0];
    cuboid.xy = cuboid.point_cloud_ptr->points[0];
    cuboid.yz = cuboid.point_cloud_ptr->points[0];
    cuboid.xz = cuboid.point_cloud_ptr->points[0];
    cuboid.xyz = cuboid.point_cloud_ptr->points[0];
    for (int i = 1; i < cuboid.point_cloud_ptr->size(); i++) {

        Point current_point = cuboid.point_cloud_ptr->points[i];;

        if (closer_cuboid_origo(cuboid.origo, current_point)) {
            cuboid.origo = current_point;
        }

        if (closer_cuboid_x(cuboid.x, current_point)) {
            cuboid.x = current_point;
        }

        if (closer_cuboid_y(cuboid.y, current_point)) {
            cuboid.y = current_point;
        }

        if (closer_cuboid_z(cuboid.z, current_point)) {
            cuboid.z = current_point;
        }

        if (closer_cuboid_xy(cuboid.xy, current_point)) {
            cuboid.xy = current_point;
        }

        if (closer_cuboid_xz(cuboid.xz, current_point)) {
            cuboid.xz = current_point;
        }

        if (closer_cuboid_yz(cuboid.yz, current_point)) {
            cuboid.yz = current_point;
        }

        if (closer_cuboid_xyz(cuboid.xyz, current_point)) {
            cuboid.xyz = current_point;
        }

    }

}

bool closer_rect_origo(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x + current_point.z;
    float new_sum = new_point.x + new_point.z;
    return new_sum < current_sum;
}

bool closer_rect_x(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x - current_point.z;
    float new_sum = new_point.x - new_point.z;
    return new_sum > current_sum;
}

bool closer_rect_z(const Point current_point, const Point &new_point) {
    float current_sum = current_point.z - current_point.x;
    float new_sum = new_point.z - new_point.x;
    return new_sum > current_sum;
}

bool closer_rect_xz(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x + current_point.z;
    float new_sum = new_point.x + new_point.z;
    return new_sum > current_sum;
}

void set_rectangle_corners(Rectangle &rectangle) {

    Cloud point_cloud = *rectangle.point_cloud_ptr;
    rectangle.origo = point_cloud.points[0];
    rectangle.x = point_cloud.points[0];
    rectangle.z = point_cloud.points[0];
    rectangle.xz = point_cloud.points[0];

    for (int i = 1; i < point_cloud.size(); i++){

        Point current_point = point_cloud.points[i];

        if (closer_rect_origo(rectangle.origo, current_point)) {
            rectangle.origo = current_point;
        }

        if (closer_rect_x(rectangle.x, current_point)) {
            rectangle.x = current_point;
        }

        if (closer_rect_z(rectangle.z, current_point)) {
            rectangle.z = current_point;
        }

        if (closer_rect_xz(rectangle.xz, current_point)) {
            rectangle.xz = current_point;
        }
    }
}

/**
   This function filters a cloud by performing a few steps:
   * It removes all noise data and outliers
   * It scales the cloud to the correct proportions
   * It makes sure the cloud has the correct rotation and position around origin
   @param cloud_in The input cloud, taken directly from a TreeD scan.
   @param cloud_out The filtered and scaled cloud is returned here.
   @param rotation The rotation the scan was taken from (in degrees).
 */

void
filter(CloudPtr cloud_in, CloudPtr cloud_out)
{
   // Use a pass through filter to remove all points outside of specific coordinates
   pcl::PassThrough<pcl::PointXYZ> pt_filter;
   pt_filter.setInputCloud(cloud_in);

   // Filter on the x axis
   // This will remove the stick the object is attached to
   pt_filter.setFilterFieldName("x");
   pt_filter.setFilterLimits(100, 511);
   pt_filter.setFilterLimitsNegative(false);
   pt_filter.filter(*cloud_out);

   // Filter on the z axis to remove the plane of noise data in the
   // beginning of the scan
   pt_filter.setInputCloud(cloud_out);
   pt_filter.setFilterFieldName("z");
   pt_filter.setFilterLimits(-1, 1);
   pt_filter.setFilterLimitsNegative(true);
   pt_filter.filter(*cloud_out);

   // Filter on the y axis
   pt_filter.setInputCloud(cloud_out);
   pt_filter.setFilterFieldName("y");
   pt_filter.setFilterLimits(250, 580);
   pt_filter.setFilterLimitsNegative(false);
   pt_filter.filter(*cloud_out);

   // Remove points that are far away from other points
   pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
   outlier_filter.setInputCloud(cloud_out);
   outlier_filter.setRadiusSearch(0.8);
   outlier_filter.setMinNeighborsInRadius(2);
   outlier_filter.filter(*cloud_out);


   // Translate the object to move the center of the object to the origin (approximately).
   // This works but should be done in a better way. Right now these values
   // will be wrong if the scanner hardware is moved.
   Eigen::Affine3f translation_transform(Eigen::Affine3f::Identity());
   translation_transform.translation() << -528.0, -346.0, 591.0;
   pcl::transformPointCloud(*cloud_out, *cloud_out, translation_transform);

   // Scale the point cloud by half to make it the correct proportions.
   // The scaling factor 0.5 assumes the scans are run with cart speed 200 mm/s.
   float scale = 0.5;
   Eigen::Affine3f scale_transform(Eigen::Affine3f::Identity());
   scale_transform.scale(Eigen::Vector3f(1, 1, scale));
   pcl::transformPointCloud(*cloud_out, *cloud_out, scale_transform);

   return;
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
            std::cout << "current depth " << current_depth << std::endl;
            if (first || current_depth < optimal_depth) {
                std::cout << "new optimal depth " << current_depth << std::endl;
                std::cout << "new optimal angle " << v << std::endl;
                optimal_cloud_ptr = current_cloud_ptr;
                optimal_depth = current_depth;
                optimal_angle = v;
                first = false;
            }
        } else {
            std::cout << "Exit code: " << srv.response.exit_code << std::endl;
            std::cout << "Error: " << srv.response.error_message << std::endl;
            std::exit(srv.response.exit_code);
        }
    }
    return optimal_angle;
}


Rectangle scan(ros::ServiceClient client, ScanService srv, degrees y_angle) {
    Rectangle rectangle;
    if (client.call(srv)) {
        if (!srv.response.exit_code) {

            CloudPtr cloud_to_save_ptr(new Cloud());
            CloudPtr cloud_temp_ptr(new Cloud());
            rectangle.pov_angle = y_angle;
            message_to_cloud(srv.response.point_cloud, cloud_temp_ptr);
            filter(cloud_temp_ptr, cloud_to_save_ptr);
            rectangle.point_cloud_ptr = cloud_to_save_ptr;
            set_rectangle_corners(rectangle);

            if (!is_rectangle(rectangle)) {
                std::cout << "ERROR: Did not recognize image as a a rectangle" << std::endl;
            }
        } else {
            std::cout << "Service code error exit code: " << srv.response.exit_code << std::endl;
            std::cout << "Error message: " << srv.response.error_message << std::endl;
            std::exit(srv.response.exit_code);
        }

        return rectangle;
    }
}

std::vector<Rectangle>
scan_object(ros::ServiceClient client, const unsigned int accuracy) {
    degrees start = get_start_angle(client, accuracy);
    std::cout << "start at angle: " << start << std::endl;
    // Vector with scanData objects. scanData objects consist of one point cloud and the angles for the rotation board.
    std::vector<Rectangle> scans;
    ScanService srv;
    degrees x_angle = 0;
    degrees y_angle = 0;
    srv.request.x_angle = x_angle;
    Rectangle rectangle;
    for (int i=0; i < 4; i++) {
        y_angle = start + i*90;
        if (y_angle >= 360) {
            y_angle -= 360;
        }
        srv.request.y_angle = y_angle;
        rectangle = scan(client, srv, y_angle);
        scans.push_back(rectangle);
    }

    return scans;
}

Cuboid
generate_cuboid(const millimeter width, const millimeter height, const millimeter depth) {

    millimeter step_size = 0.5;
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
 * This function first rotates the Cubiod to match the Rectangle. It then translates the Cuboid to match the Rectangles
 * coodinates. And finaly runs the ICP algorithm to register the pointclouds.
 *
 * @param target    The Cuboid object containing the target pointcloud
 * @param source    The Rectangle object containing the source pointcloud
 * @return True if ICP converges false otherwise
 */
bool
register_point_clouds_icp(Cuboid target, Rectangle source) {

    const CloudPtr source_cloud = source.point_cloud_ptr;
    CloudPtr target_cloud = target.point_cloud_ptr;
    CloudPtr target_cloud_new(new Cloud);
    CloudPtr final_cloud(new Cloud);


    Eigen::Affine3f TransformRotate = Eigen::Affine3f::Identity();
    Eigen::Affine3f TransformTranslate = Eigen::Affine3f::Identity();

    // Compute rotation
    degrees angle = source.pov_angle - target.pov_angle; // The angle of rotation in radians
    float theta = angle*(M_PI/180);
    TransformRotate.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));

    // Executing the rotation
    CloudPtr transformed_cloud(new Cloud);
    pcl::transformPointCloud(*target_cloud, *target_cloud, TransformRotate);
    target.pov_angle = theta;

    // Update target
    //target.point_cloud_ptr = transformed_cloud;
    set_cuboid_corners(target);

    // Compute the translation
    float translation_x = source.origo.x - target.origo.x;
    float translation_y = source.origo.y - target.origo.y;
    float translation_z = source.origo.z - target.origo.z;


    // Execute the translation
    TransformTranslate.translation() << translation_x, translation_y, translation_z;
    pcl::transformPointCloud(*target_cloud, *target_cloud, TransformTranslate);

    //Update target
    set_cuboid_corners(target);

    // Start ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    // Parameters for the ICP algorithm
    icp.setInputTarget(source_cloud);
    icp.setInputSource(target_cloud);
    icp.setMaximumIterations(25);
    icp.setTransformationEpsilon(1e-7);
    icp.setMaxCorrespondenceDistance(3);
    //icp.setEuclideanFitnessEpsilon(1);
    //icp.setRANSACOutlierRejectionThreshold(1);

    icp.align(*target_cloud);

    if (icp.hasConverged()) {
        std::cout << "ICP converged." << std::endl
                  << "The score is " << icp.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
        Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation();
        std::cout << "trans %n" << transformationMatrix << std::endl;

        pcl::transformPointCloud(*target_cloud, *target_cloud_new, transformationMatrix);

        *final_cloud = *source_cloud + *target_cloud;

        pcl::io::savePCDFileASCII("ICP_result.pcd", *final_cloud);
        return true;
    }

    else std::cout << "ICP did not converge." << std::endl;

    Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation();
    std::cout << "trans %n" << transformationMatrix << std::endl;

    pcl::transformPointCloud(*target_cloud, *target_cloud_new, transformationMatrix);

    *final_cloud = *source.point_cloud_ptr + *target.point_cloud_ptr;

    pcl::io::savePCDFileASCII("ICP_result.pcd", *final_cloud);

    return false;

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

    std::cout << "get point cloud" << std::endl;
    ros::NodeHandle node_handle;
    ros::ServiceClient client = node_handle.serviceClient<ScanService>("wrapper_scan");
    std::vector<Rectangle> scans;

    scans = scan_object(client, req.accuracy);

    millimeter width = std::abs(scans[0].z.z - scans[0].origo.z);
    millimeter height = std::abs(scans[0].x.x - scans[0].origo.x);
    millimeter depth = std::abs(scans[1].z.z - scans[1].origo.z);
    std::cout << "w: " << width << " h: " << height << " d: " << depth << std::endl;
    std::cout << "0x: " << scans[0].x << " 0z: " << scans[0].z <<  " 0origo: " << scans[0].origo << std::endl;
    std::cout << "1x: " << scans[1].x << " 1z: " << scans[1].z << " 1origo: " << scans[1].origo  << std::endl;
    Cuboid cuboid = generate_cuboid(width, height, depth);
    std::cout << "co: " << cuboid.origo << " cx: " << cuboid.x << " cy: " << cuboid.y << " cz: " << cuboid.z << std::endl;


    for (int i = 0; i < scans.size(); i++){
        std::stringstream ss;
        ss << "rect_" << i << ".pcd";
        std::string filename = ss.str();
        pcl::io::savePCDFile(filename, *scans[i].point_cloud_ptr);
    }
    pcl::io::savePCDFile("cuboid.pcd", *cuboid.point_cloud_ptr);

    for (int i = 0; i < scans.size(); i++){
       if(!register_point_clouds_icp(cuboid, scans[i])){
           std::cout << "ICP failed trying to register point cloud nr " <<i<<std::endl;
       }
        else{
           std::cout << "Registration of point cloud " << i << " was successful" << std::endl;

       }
    }

    return true;
}


bool
test_scan(TestScan::Request &req, TestScan::Response &resp)
{
    /*std::cout << "test scan" << std::endl;
    ros::NodeHandle node_handle;
    ros::ServiceClient client = node_handle.serviceClient<ScanService>("wrapper_scan");
    ScanService srv;
    CloudPtr cloud_to_save_ptr (new Cloud());
    CloudPtr cloud_temp_ptr (new Cloud());
    srv.request.x_angle = 0;
    srv.request.y_angle = 0;
    if (client.call(srv) && !srv.response.exit_code){
        message_to_cloud(srv.response.point_cloud, cloud_temp_ptr);
        filter(cloud_temp_ptr, cloud_to_save_ptr);
        pcl::io::savePCDFile(req.filename, *cloud_to_save_ptr);
    }
    return true;*/

    // Crerate target and source CloudPtr
    CloudPtr target_cloud_read(new Cloud);
    CloudPtr source0_cloud_read(new Cloud);
    CloudPtr source1_cloud_read(new Cloud);
    CloudPtr source2_cloud_read(new Cloud);
    CloudPtr source3_cloud_read(new Cloud);
    CloudPtr target_cloud_new(new Cloud);
    CloudPtr final_cloud(new Cloud);

    std::vector<Rectangle> scans;

    // Read Target pointcloud
    if (pcl::io::loadPCDFile("cuboid.pcd", *target_cloud_read) == -1) {
        //std::cout << argv[0] << std::endl;
        PCL_ERROR ("Couldn't read first file! \n");
        return (-1);
    }
    // Create Cuboid object
    Cuboid Target;
    Target.point_cloud_ptr = target_cloud_read;
    set_cuboid_corners(Target);
    // Read source pointcloud
    if (pcl::io::loadPCDFile("rect_0.pcd", *source0_cloud_read) == -1) {
        PCL_ERROR ("Couldn't read second input file! \n");
        return (-1);
    }
    // Create Rectangle object
    Rectangle Source0;
    Source0.point_cloud_ptr = source0_cloud_read;
    set_rectangle_corners(Source0);
    scans.push_back(Source0);

    if (pcl::io::loadPCDFile("rect_1.pcd", *source1_cloud_read) == -1) {
        PCL_ERROR ("Couldn't read second input file! \n");
        return (-1);
    }
    Rectangle Source1;
    Source1.point_cloud_ptr = source1_cloud_read;
    set_rectangle_corners(Source1);
    scans.push_back(Source1);
    if (pcl::io::loadPCDFile("rect_2.pcd", *source2_cloud_read) == -1) {
        PCL_ERROR ("Couldn't read second input file! \n");
        return (-1);
    }
    Rectangle Source2;
    Source2.point_cloud_ptr = source2_cloud_read;
    set_rectangle_corners(Source2);
    scans.push_back(Source2);
    if (pcl::io::loadPCDFile("rect_3.pcd", *source3_cloud_read) == -1) {
        PCL_ERROR ("Couldn't read second input file! \n");
        return (-1);
    }
    Rectangle Source3;
    Source3.point_cloud_ptr = source3_cloud_read;
    set_rectangle_corners(Source3);
    scans.push_back(Source3);



    for(int i = 0; i < 2; i++){


        Eigen::Affine3f TransformRotate = Eigen::Affine3f::Identity();
        Eigen::Affine3f TransformTranslate = Eigen::Affine3f::Identity();

        // Compute rotation
        degrees angle = 90; // The angle of rotation in radians
        float theta = angle*(M_PI/180);
        TransformRotate.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));

        // Executing the rotation
        CloudPtr transformed_cloud(new Cloud);
        pcl::transformPointCloud(*Target.point_cloud_ptr, *Target.point_cloud_ptr, TransformRotate);
        Target.pov_angle = theta;

        // Update target
        //target.point_cloud_ptr = transformed_cloud;
        set_cuboid_corners(Target);

        // Compute the translation
        float translation_x;
        float translation_y;
        float translation_z;
        if(i == 1){
            std::cout<<i<<std::endl;
            translation_x = scans[i].origo.x - Target.xy.x;
            translation_y = scans[i].origo.y - Target.xy.y;
            translation_z = scans[i].origo.z - Target.xy.z;
        }

        else{
            translation_x = scans[i].origo.x - Target.origo.x;
            translation_y = scans[i].origo.y - Target.origo.y;
            translation_z = scans[i].origo.z - Target.origo.z;
        }




        std::cout<<"translation: "<<translation_x<<" , "<<translation_y<<" , "<<translation_z<<std::endl;

        // Execute the translation
        TransformTranslate.translation() << translation_x, translation_y, translation_z;
        pcl::transformPointCloud(*Target.point_cloud_ptr, *Target.point_cloud_ptr, TransformTranslate);

        //Update target
        set_cuboid_corners(Target);

        // Start ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

        // Parameters for the ICP algorithm
        icp.setInputTarget(scans[i].point_cloud_ptr);
        icp.setInputSource(Target.point_cloud_ptr);
        icp.setMaximumIterations(25);
        icp.setTransformationEpsilon(1e-7);
        icp.setMaxCorrespondenceDistance(3);
        //icp.setEuclideanFitnessEpsilon(1);
        //icp.setRANSACOutlierRejectionThreshold(1);

        icp.align(*Target.point_cloud_ptr);

        if (icp.hasConverged()) {
            std::cout << "ICP converged." << std::endl
                      << "The score is " << icp.getFitnessScore() << std::endl;
            //std::cout << "Transformation matrix:" << std::endl;
            //std::cout << icp.getFinalTransformation() << std::endl;
            Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation();
            //std::cout << "trans %n" << transformationMatrix << std::endl;

            pcl::transformPointCloud(*Target.point_cloud_ptr, *target_cloud_new, transformationMatrix);

            *final_cloud = *scans[i].point_cloud_ptr + *Target.point_cloud_ptr;

            pcl::io::savePCDFileASCII("ICP_result_pass.pcd", *final_cloud);
            //return true;
            Target.point_cloud_ptr = final_cloud;
            set_cuboid_corners(Target);
        }

        else std::cout << "ICP did not converge." << std::endl;

        Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation();
        //std::cout << "trans %n" << transformationMatrix << std::endl;

        pcl::transformPointCloud(*Target.point_cloud_ptr, *target_cloud_new, transformationMatrix);

        *final_cloud = *scans[i].point_cloud_ptr + *Target.point_cloud_ptr;

        pcl::io::savePCDFileASCII("ICP_result_fail.pcd", *final_cloud);
        Target.point_cloud_ptr = final_cloud;
        set_cuboid_corners(Target);

        //return false;
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

    ros::ServiceServer testService = node_handle.advertiseService("test_scan", test_scan);
    ROS_INFO("Ready to start a test scan");

    ros::spin();
    return (0);
}
