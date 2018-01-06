#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <pcl/point_cloud.h>
#include <pcl/registration/correspondence_estimation.h>
#include "Core/filtering.h"

namespace Core {
pcl::Correspondences correspKD(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr, const float &maxDist);
pcl::Correspondences correspRpoly(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::Correspondences, const int &, const int &, const float &);
pcl::Correspondences correspRdist(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::Correspondences,const float &);
pcl::Correspondences correspRmeddist(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::Correspondences,const double &);
pcl::Correspondences correspR121(pcl::Correspondences);
pcl::Correspondences correspRransac(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::Correspondences, const int &, const float &);
pcl::Correspondences correspRsurfacenorm(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::Correspondences, const float &);
pcl::Correspondences correspRboudary(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::Correspondences );
pcl::Correspondences fullCorresp(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr, const int &, const float &, const float &, const float &thres, const float &);
Eigen::Matrix4d icpLLSp2s(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr , const int &, const float &, const float &, const float &, const float &);
Eigen::Matrix4d icpSVD(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr , const int &, const float &, const float &, const float &, const float &);
Eigen::Matrix4d icpLMp2p(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr , const int &, const float &, const float &, const float &, const float &);
Eigen::Matrix4d icpLMp2s(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr , const int &, const float &, const float &, const float &, const float &);
void fullRegister(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vec, const int &icpMode, const float &maxDepthChange, const float &smoothSize, const int &iter, const float &maxDist, const float &medFac, const float &thres, const float &in_thres );
pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullRegisterIncremental(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &vec, const int &icpMode, const float &maxDepthChange, const float &smoothSize, const int &iter, const float &maxDist, const float &medFac, const float &thres, const float &in_thres );
Eigen::Matrix4d pairRegister(pcl::PointCloud<pcl::PointNormal>::Ptr src, pcl::PointCloud<pcl::PointNormal>::Ptr target, const int &icpMode, const int &iter, const float &maxDist, const float &medFac, const float &thres, const float &in_thres);}

#endif // REGISTRATION_H
