#ifndef MATHTOOLS_H
#define MATHTOOLS_H

#include <pcl/point_cloud.h>
#include <pcl/filters/normal_space.h>
namespace Core {
 pcl::PointCloud<pcl::PointNormal>::Ptr normal2PointNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr , pcl::PointCloud<pcl::Normal>::Ptr);
}

#endif // MATHTOOLS_H
