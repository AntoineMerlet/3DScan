#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <pcl/point_cloud.h>
#include <pcl/registration/correspondence_estimation.h>

namespace Core {
pcl::Correspondences correspKD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const float &);
pcl::Correspondences correspRpoly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const int &);

}

#endif // REGISTRATION_H
