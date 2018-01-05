#include "registration.h"
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_poly.h>
#include <pcl/point_cloud.h>
namespace Core {
pcl::Correspondences correspKD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, const float &maxDist)
{
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB> corresp_kdtree;
    pcl::Correspondences corresp;
    corresp_kdtree.setInputSource (src);
    corresp_kdtree.setInputTarget (target);
    corresp_kdtree.determineCorrespondences (corresp, maxDist);
    return corresp;
}

pcl::Correspondences correspRpoly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, pcl::Correspondences corresp, const int &iter, const int &cardi, const float &simiThre)
{
    pcl::registration::CorrespondenceRejectorPoly<pcl::PointXYZ> rejector_poly;
    pcl::Correspondences correspRes;
    rejector_poly.setCardinality(cardi);
    rejector_poly.setIterations(iter);
    rejector_poly.setSimilarityThreshold(simiThre);
    rejector_poly.setInputSource(src);
    rejector_poly.setInputTarget(target);
    rejector_poly.setInputCorrespondences (corresp);
    rejector_poly.getCorrespondences (*correspRes);
    return correspRes;
}
}
