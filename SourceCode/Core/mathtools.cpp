#include "mathtools.h"
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include "IO/logger.h"

namespace Core {
/// @author: Antoine Merlet
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Converts normal data (sinlge) to PC + normals
/// @param cloud_in The point cloud xyz coordinates and RGB components
/// @param normals: the normals for this point cloud
/// @return The point cloud and its normals (loss of RGB info)
pcl::PointCloud<pcl::PointNormal>::Ptr normal2PointNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);

    cloud_with_normals->width = 640;
    cloud_with_normals->height = 480;
    cloud_with_normals->points.resize(cloud_with_normals->width * cloud_with_normals->height);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_xyz, *cloud_in);
    pcl::concatenateFields (*cloud_xyz, *normals, *cloud_with_normals);

    return cloud_with_normals;
}

/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Surface normal estimation on organized data using integral images
/// @param cloud_in: The point cloud to compute normals from
/// @param maxDepthChange: the depth change threshold for computing object borders based on depth changes
/// @param smoothSize: smooth size factor which influences the size of the area used to smooth normals (depth dependent if useDepthDependentSmoothing is true)
/// @return normals
pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const float &radius)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setRadiusSearch (radius);
    norm_est.setInputCloud (cloud_in);
    norm_est.compute (*normals);

    return normals;
}

/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Surface normal estimation on organized data using integral images
/// @param cloud_in: The point cloud to compute normals from
/// @param maxDepthChange: the depth change threshold for computing object borders based on depth changes
/// @param smoothSize: smooth size factor which influences the size of the area used to smooth normals (depth dependent if useDepthDependentSmoothing is true)
/// @return Cloud with its normals.
pcl::PointCloud<pcl::PointNormal>::Ptr getNormalPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const float &radius)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr normals (new pcl::PointCloud<pcl::PointNormal>);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> norm_est;
    norm_est.setRadiusSearch (radius);
    norm_est.setInputCloud (cloud_in);
    norm_est.compute (*normals);

    return normals;
}
}
