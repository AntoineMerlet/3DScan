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
pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const float &maxDepthChange, const float &smoothSize )
{
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>normest;

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    normals->width = 640;
    normals->height = 480;
    normals->points.resize(normals->width * normals->height);

    normest.setNormalEstimationMethod(normest.AVERAGE_DEPTH_CHANGE);
    normest.setMaxDepthChangeFactor(maxDepthChange); // val = 0.05
    normest.setDepthDependentSmoothing(true);
    normest.setNormalSmoothingSize(smoothSize); // val = 10.0f
    normest.setInputCloud(cloud_in);

    normest.compute(*normals);

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
pcl::PointCloud<pcl::PointNormal>::Ptr getNormalPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const float &maxDepthChange, const float &smoothSize )
{
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>normest;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    cloud_in->width = 640;
    cloud_in->height = 480;
    cloud_in->points.resize(cloud_in->width * cloud_in->height);

    normest.setNormalEstimationMethod(normest.AVERAGE_DEPTH_CHANGE);
    normest.setMaxDepthChangeFactor(maxDepthChange);
    normest.setNormalSmoothingSize(smoothSize);
    normest.setInputCloud(cloud_in);
    normest.compute(*normals);

    LOG("Normals Done. Now " + std::to_string(normals->size()));

    return normal2PointNormal(cloud_in,normals);
}
}
