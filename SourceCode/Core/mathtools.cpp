#include "mathtools.h"

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_xyz, *cloud_in);
    pcl::concatenateFields (*cloud_xyz, *normals, *cloud_with_normals);
    return cloud_with_normals;
}
}
