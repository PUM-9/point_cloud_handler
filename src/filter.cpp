#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>

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
filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, int rotation)
{
    // Use a pass through filter to remove all points outside of specific coordinates
    pcl::PassThrough<pcl::PointXYZ> pt_filter;
    pt_filter.setInputCloud(cloud_in);

    // Filter on the x axis
    // This will remove the stick the object is attached to
    pt_filter.setFilterFieldName("x");
    pt_filter.setFilterLimits(100, 528);
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
    pt_filter.setFilterLimits(100, 580);
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

    // Rotate the object around the x axis to match the objects real world rotation
    Eigen::Matrix3f rotation_matrix(Eigen::AngleAxisf((rotation*M_PI) / 180, Eigen::Vector3f::UnitX()));
    Eigen::Affine3f rotation_transform(Eigen::Affine3f::Identity());
    rotation_transform.rotate(rotation_matrix);
    pcl::transformPointCloud(*cloud_out, *cloud_out, rotation_transform);

    return;
}
