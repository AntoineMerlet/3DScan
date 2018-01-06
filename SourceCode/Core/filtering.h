#ifndef FILTERING_H
#define FILTERING_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <vector>
#include "logger.h"

namespace Core {
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const float &, const float &, const float &);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr bilateralFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const float &, const float &);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr bilateralupsamplerRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const float &, const float &);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr medianFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const int &, const float &);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const int &);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr normalSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const unsigned int &, const unsigned int &, const float &, const float &);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr covarianceSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const unsigned int &, const float &, const float &);
pcl::PointCloud<pcl::PointNormal>::Ptr getNormalPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const float &, const float &);
pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const float &, const float &);
}

#endif // FILTERING_H
