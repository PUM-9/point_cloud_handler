#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/**
 * Removes the stick holding the object from a scan. This function assumes that
 * the input point cloud has already been filtered of any noise besides the stick.
 *
 * The function will first try to divide to point cloud into separate clusters. If it
 * can split the cloud in exactly two clusters it will assume the lowest cluster is the stick
 * and remove it. If it can't find exactly two clusters it will fall back on a basic pass
 * through filter on the x axis.
 *
 * @param cloud_in The input point cloud
 * @param cloud_out The resulting point cloud with the stick removed
 * @return Returns true if a clustering filter was possible and false if a pass
 *         through filter was used
 */
bool remove_stick(PointCloud::ConstPtr cloud_in, PointCloud::Ptr cloud_out)
{
    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    search_tree->setInputCloud(cloud_in);

    // Create the object for extracting cluster in cloud_in
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction;
    cluster_extraction.setInputCloud(cloud_in);

    // Set the maximum distance between two points in a cluster to 4 mm
    cluster_extraction.setClusterTolerance(4.0);

    // Set a cluster to be between 10 and 25000 points
    cluster_extraction.setMinClusterSize(10);
    cluster_extraction.setMaxClusterSize(25000);

    // Perform the euclidean cluster extraction algorithm
    cluster_extraction.setSearchMethod(search_tree);
    cluster_extraction.extract(cluster_indices);

    // Generate all the individual cluster point clouds
    std::vector<PointCloud::Ptr> clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        PointCloud::Ptr cluster(new PointCloud);

        // For every list of indices, add all the points to a point cloud
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cluster->points.push_back(cloud_in->points[*pit]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    
    if (!clusters.empty()) {
        std::vector<PointCloud::Ptr> good_clusters;

        for (size_t i = 0; i < clusters.size(); ++i) {
	    std::stringstream ss;
	    ss << "cluster-" << i << ".pcd";
	    pcl::io::savePCDFile(ss.str(), *clusters.at(i));

	    Eigen::Vector4f centroid(Eigen::Vector4f::Zero());
	    pcl::compute3DCentroid(*clusters.at(i), centroid);
	  
	    std::cout << "Centroid: " << std::endl << centroid << std::endl << std::endl;

	    if (centroid(0, 0) < 515) {
	        good_clusters.push_back(clusters.at(i));
	    }
	}
	
	std::cout << "Good clusters: " << good_clusters.size() << std::endl;
	for (size_t i = 0; i < good_clusters.size(); ++i) {
	    std::cout << "Adding cluster " << i << std::endl;
	    if (i == 0) {
	        *cloud_out = *good_clusters.at(i);
	    } else {
	        *cloud_out += *good_clusters.at(i);
	    }
	}
	
	return true;
    } else {
        *cloud_out = *cloud_in;
        return false;
    }


    // If we have exactly two clusters one will be the object and one will be the stick.
    // Look at which object is higher up to figure out which is which.
    if (clusters.size() == 2) {
        // Compute the centroids for both clusters to find out where they are located
        Eigen::Vector4f first_centroid(Eigen::Vector4f::Zero());
        pcl::compute3DCentroid(*clusters.at(0), first_centroid);

        Eigen::Vector4f second_centroid(Eigen::Vector4f::Zero());
        pcl::compute3DCentroid(*clusters.at(1), second_centroid);

        // Set the correct output cloud based on the x (height) value of the centroids
        if (first_centroid(0, 0) > second_centroid(0, 0)) {
            *cloud_out = *clusters.at(1);
        } else {
            *cloud_out = *clusters.at(0);
        }

        return true;
    } else {
        // We can't figure out where the stick is by clustering, perform a simple pass through filter instead
        pcl::PassThrough<pcl::PointXYZ> pt_filter;
        pt_filter.setInputCloud(cloud_in);
        pt_filter.setFilterFieldName("x");
        pt_filter.setFilterLimits(100, 510);
        pt_filter.setFilterLimitsNegative(false);
        pt_filter.filter(*cloud_out);
        
        return false;
    }
}

/**
 * This function filters a cloud by performing a few steps:
 * * It removes all noise data and outliers
 * * It scales the cloud to the correct proportions
 * * It makes sure the cloud has the correct rotation and position around origin
 * @param cloud_in The input cloud, taken directly from a TreeD scan.
 * @param cloud_out The filtered and scaled cloud is returned here.
 * @param rotation The rotation the scan was taken from (in degrees).
 */
void filter(PointCloud::ConstPtr cloud_in, PointCloud::Ptr cloud_out, int rotation)
{
    // Use a pass through filter to remove all points outside of specific coordinates
    pcl::PassThrough<pcl::PointXYZ> pt_filter;
    pt_filter.setInputCloud(cloud_in);

    // Do initial rough filtering on the x axis to remove noise data
    pt_filter.setFilterFieldName("x");
    pt_filter.setFilterLimits(310, 568);
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

    // Remove the stick the object is attached to
    remove_stick(cloud_out, cloud_out);
    
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
