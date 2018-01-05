#include "mathtools.h"

namespace Core {
pcl::PointCloud<pcl::PointNormal>::Ptr normal2PointNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_xyz, *cloud_in);
    pcl::concatenateFields (*cloud_xyz, *normals, *cloud_with_normals);
    return cloud_with_normals;
}
}
