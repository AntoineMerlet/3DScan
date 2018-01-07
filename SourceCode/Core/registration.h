#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <pcl/point_cloud.h>
#include <pcl/registration/correspondence_estimation.h>
#include "Core/filtering.h"

namespace Core {
pcl::Correspondences correspKD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr,pcl::PointCloud<pcl::Normal>::Ptr,pcl::PointCloud<pcl::Normal>::Ptr, const float &);
pcl::Correspondences correspRpoly(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::Correspondences, const int &, const int &, const float &);
pcl::Correspondences correspRdist(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::Correspondences,const float &);
pcl::Correspondences correspRmeddist(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::Correspondences,const double &);
pcl::Correspondences correspR121(pcl::Correspondences);
pcl::Correspondences correspRransac(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::Correspondences, const int &, const float &);
pcl::Correspondences correspRsurfacenorm(pcl::PointCloud<pcl::PointNormal>::Ptr,pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::Correspondences, const float &);
pcl::Correspondences correspRboudary(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::Correspondences );
pcl::Correspondences fullCorresp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const int &, const float &, const float &, const float &, const float &);
Eigen::Matrix4d icpLLSp2s(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr , const int &, const float &, const float &, const float &, const float &);
Eigen::Matrix4d icpSVD(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr , const int &, const float &, const float &, const float &, const float &);
Eigen::Matrix4d icpLMp2p(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr , const int &, const float &, const float &, const float &, const float &);
Eigen::Matrix4d icpLMp2s(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr , const int &, const float &, const float &, const float &, const float &);
void fullRegister(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vec, const int &icpMode, const float &, const int &, const float &, const float &, const float &thres, const float &);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullRegisterIncremental(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &, const int &, const float &, const int &, const float &, const float &, const float &, const float &in_thres );
Eigen::Matrix4d pairRegister(pcl::PointCloud<pcl::PointNormal>::Ptr src, pcl::PointCloud<pcl::PointNormal>::Ptr, const int &, const int &, const float &, const float &, const float &, const float &);}

#endif // REGISTRATION_H
