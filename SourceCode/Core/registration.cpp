#include "registration.h"
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_poly.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/point_cloud.h>
namespace Core {
/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Function determines  correspondences between target and query point sets/features
/// @param src: the input point cloud source
/// @param target: target point cloud
/// @param maxDist: maximum allowed distance between correspondences
/// @return the found correspondences (index of query point, index of target point, distance)
pcl::Correspondences correspKD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, const float &maxDist)
{
    LOG("Downsampling ...")
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB> corresp_kdtree;
    pcl::Correspondences corresp;
    corresp_kdtree.setInputSource (src);
    corresp_kdtree.setInputTarget (target);
    corresp_kdtree.determineCorrespondences (corresp, maxDist);
    LOG("Correspondences")
    return corresp;
}


/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Function implements a correspondence rejection method that exploits low-level and pose-invariant geometric constraints between two point sets.
/// @param src: the input point cloud source
/// @param target: target point cloud
/// @param corresp:the found correspondences (index of query point, index of target point, distance)/the set of initial correspondences given
/// @param iter: number of iterations
/// @param cardi: polygon cardinality
/// @param simiThre: similarity threshold.Set the similarity threshold in [0,1[ between edge lengths, where 1 is a perfect match.
/// @return the resultant filtered set of remaining correspondences

pcl::Correspondences correspRpoly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, pcl::Correspondences corresp, const int &iter, const int &cardi, const float &simiThre)
{
    LOG("Correspondence rejection is implementing")
    pcl::registration::CorrespondenceRejectorPoly<pcl::PointXYZRGB, pcl::PointXYZRGB> rejector_poly;
    pcl::Correspondences correspRes;
    pcl::CorrespondencesConstPtr ptr(&corresp);

    rejector_poly.setCardinality(cardi);
    rejector_poly.setIterations(iter);
    rejector_poly.setSimilarityThreshold(simiThre);
    rejector_poly.setInputSource(src);
    rejector_poly.setInputTarget(target);
    rejector_poly.setInputCorrespondences(ptr);

    rejector_poly.getRemainingCorrespondences(corresp, correspRes);
    LOG("Result of remaining correspondences")
    return correspRes;
}




/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Implements a simple correspondence rejection method based on thresholding the distances between the correspondences.
/// @param src: the input point cloud source
/// @param target: target point cloud
/// @param corresp:the found correspondences (index of query point, index of target point, distance)/the set of initial correspondences given
/// @param maxDist: Distance to be used as maximum distance between correspondences. Correspondences with larger distances are rejected.
/// @return the resultant filtered set of remaining correspondences
pcl::Correspondences correspRdist(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, pcl::Correspondences corresp,const float &maxDist)
{
    LOG("Correspondence rejection based on thresholding is implementing")
    pcl::registration::CorrespondenceRejectorDistance rejector_dist;
    pcl::Correspondences correspRes;
    rejector_dist.setInputSource<pcl::PointXYZRGB>(src);
    rejector_dist.setInputTarget<pcl::PointXYZRGB>(target);
    rejector_dist.setMaximumDistance(maxDist);
    rejector_dist.getRemainingCorrespondences(corresp, correspRes);
    LOG("Result");
    return correspRes;
}




/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Implements a simple correspondence rejection method based on thresholding the distances between the correspondences.
/// @param src: the input point cloud source
/// @param target: target point cloud
/// @param corresp:the found correspondences (index of query point, index of target point, distance)/the set of initial correspondences given
/// @param medFact: Set the factor for correspondence rejection.
/// @return the resultant filtered set of remaining correspondences
pcl::Correspondences correspRmeddist(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, pcl::Correspondences corresp,const double &medFact)
{
    LOG("Correspondence rejection based on thresholding is implementing");
    pcl::registration::CorrespondenceRejectorMedianDistance rejector_meddist;
    pcl::Correspondences correspRes;
    rejector_meddist.setInputSource<pcl::PointXYZRGB>(src);
    rejector_meddist.setInputTarget<pcl::PointXYZRGB>(target);
    rejector_meddist.setMedianFactor(medFact);
    rejector_meddist.getRemainingCorrespondences(corresp, correspRes);
    LOG("Result");
    return correspRes;
}



/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Method based on eliminating duplicate match indices in the correspondences.
/// @param corresp:the found correspondences (index of query point, index of target point, distance)/the set of initial correspondences given
/// @return the resultant filtered set of remaining correspondences
pcl::Correspondences correspR121(pcl::Correspondences corresp)
{
    LOG("Correspondence with eliminating duplicate matching");
    pcl::registration::CorrespondenceRejectorOneToOne rejector_121;
    pcl::Correspondences correspRes;
    rejector_121.getRemainingCorrespondences(corresp, correspRes);
    LOG("Result")
    return correspRes;
}



/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief correspondence rejection using Random Sample Consensus to identify inliers (and reject outliers)
/// @param src: the input point cloud source
/// @param target: target point cloud
/// @param corresp:the found correspondences (index of query point, index of target point, distance)/the set of initial correspondences given
/// @param iter: Maximum number if iterations to run
/// @param in_thres: Distance threshold in the same dimension as source and target data sets.
/// @return the resultant filtered set of remaining correspondences
pcl::Correspondences correspRransac(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, pcl::Correspondences corresp, const int &iter, const float &in_thres)
{

    LOG("Correspondence rejection using RANSAC")
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> rejector_ransac;
    pcl::Correspondences correspRes;

    rejector_ransac.setInputSource(src);
    rejector_ransac.setInputTarget(target);
    rejector_ransac.setMaxIterations(iter);
    rejector_ransac.setInlierThreshold(in_thres);
    rejector_ransac.setSaveInliers(true);

    rejector_ransac.getRemainingCorrespondences(corresp, correspRes);

    LOG("Result")
    return correspRes;
}



/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief implements a simple correspondence rejection method based on the angle between the normals at correspondent points.
/// @param src: the input point cloud source
/// @param target: target point cloud
/// @param corresp:the found correspondences (index of query point, index of target point, distance)/the set of initial correspondences given
/// @param maxDepthChange:
/// @param smoothSize:
/// @param thres:cosine of the thresholding angle between the normals for rejection
/// @return the resultant filtered set of remaining correspondences
pcl::Correspondences correspRsurfacenorm(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, pcl::Correspondences corresp, const float &maxDepthChange, const float &smoothSize, const float &thres)
{

    LOG("Correspondence rejection by angle is implementing ")
    pcl::registration::CorrespondenceRejectorSurfaceNormal rejector_norm;
    pcl::Correspondences correspRes;

    rejector_norm.setThreshold(thres);
    rejector_norm.setInputSource<pcl::PointXYZRGB>(src);
    rejector_norm.setInputTarget<pcl::PointXYZRGB>(target);
    rejector_norm.setInputNormals<pcl::PointNormal, pcl::PointNormal>(getNormalPoints(src, maxDepthChange, smoothSize));
    rejector_norm.setTargetNormals<pcl::PointNormal, pcl::PointNormal>(getNormalPoints(target, maxDepthChange, smoothSize));
    rejector_norm.getRemainingCorrespondences(corresp, correspRes);
    LOG("Result")
    return correspRes;
}




void fullregister(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &vec )
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, source;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::reverse_iterator it = vec.rbegin();

    for (it; it != vec.rend() -1; ++it)
    {
        target = *(it + 1);
        source = *it;
        pairregister(target,source);
    }
}


void pairregister(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source)
{

}
}
