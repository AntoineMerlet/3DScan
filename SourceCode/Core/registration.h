#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <pcl/point_cloud.h>
#include <pcl/registration/correspondence_estimation.h>
#include "Core/filtering.h"

namespace Core {
pcl::Correspondences correspKD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const float &);
pcl::Correspondences correspRpoly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const int &);
pcl::Correspondences correspRdist(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, pcl::Correspondences corresp,const float &);
pcl::Correspondences correspRmeddist(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, pcl::Correspondences corresp,const double &);
pcl::Correspondences correspR121(pcl::Correspondences);
pcl::Correspondences correspRransac(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::Correspondences, const int &iter, const float &);
pcl::Correspondences correspRsurfacenorm(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::Correspondences, const float &, const float &, const float &);


void fullregister(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &);
void pairregister(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &);
}

#endif // REGISTRATION_H
