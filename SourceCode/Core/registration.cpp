#include "registration.h"
#include "Core/mathtools.h"
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_poly.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_organized_boundary.h>

#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/default_convergence_criteria.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>

#include <pcl/point_cloud.h>
namespace Core {

/// @author: Gulnur Ungan (Antoine Merlet)
/// @date: 05-01-2018 (06-01-2018)
/// @version 1.0 (1.1)
///
/// @brief Function determines  correspondences between target and query point sets/features
/// @param src: the input point cloud source
/// @param target: target point cloud
/// @param maxDist: maximum allowed distance between correspondences
/// @return the found correspondences (index of query point, index of target point, distance)
pcl::Correspondences correspKD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, pcl::PointCloud<pcl::Normal>::Ptr srcN, pcl::PointCloud<pcl::Normal>::Ptr targetN)
{
    LOG("Finding correspondences. src: " + std::to_string(src->size()) + " points, target: " + std::to_string(target->size()) + " points");
    pcl::registration::CorrespondenceEstimationBackProjection<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::Normal> corr_est;
    pcl::Correspondences corresp;

    corr_est.setInputSource(src);
    corr_est.setSourceNormals(srcN);
    corr_est.setInputTarget(target);
    corr_est.setTargetNormals(targetN);

    corr_est.determineReciprocalCorrespondences(corresp);
    LOG("Found " + std::to_string(corresp.size()) + " correspondences.");
    return corresp;
}

/// @author: Gulnur Ungan (Antoine Merlet)
/// @date: 05-01-2018 (06-01-2018)
/// @version 1.0 (1.1)
///
/// @brief Function implements a correspondence rejection method that exploits low-level and pose-invariant geometric constraints between two point sets.
/// @param src: the input point cloud source
/// @param target: target point cloud
/// @param corresp: the found correspondences (index of query point, index of target point, distance)/the set of initial correspondences given
/// @param iter: number of iterations
/// @param cardi: polygon cardinality
/// @param simiThre: similarity threshold.Set the similarity threshold in [0,1[ between edge lengths, where 1 is a perfect match.
/// @return the resultant filtered set of remaining correspondences
pcl::Correspondences correspRpoly(pcl::PointCloud<pcl::PointNormal>::Ptr src, pcl::PointCloud<pcl::PointNormal>::Ptr target, pcl::Correspondences corresp, const int &iter, const int &cardi, const float &simiThre)
{
    LOG("Polygon correspondence rejection on " + std::to_string(corresp.size()) + " correspondences" );
    pcl::registration::CorrespondenceRejectorPoly<pcl::PointNormal, pcl::PointNormal> rejector_poly;
    pcl::Correspondences correspRes;
    pcl::CorrespondencesConstPtr ptr(&corresp);

    rejector_poly.setCardinality(cardi);
    rejector_poly.setIterations(iter);
    rejector_poly.setSimilarityThreshold(simiThre);
    rejector_poly.setInputSource(src);
    rejector_poly.setInputTarget(target);
    rejector_poly.setInputCorrespondences(ptr);

    rejector_poly.getRemainingCorrespondences(corresp, correspRes);
    LOG("Kept " + std::to_string(correspRes.size()) + " correspondences (" + std::to_string( (correspRes.size() / corresp.size()) * 100) + "%)");
    return correspRes;
}


/// @author: Gulnur Ungan (Antoine Merlet)
/// @date: 05-01-2018 (06-01-2018)
/// @version 1.0 (1.1)
///
/// @brief Implements a simple correspondence rejection method based on thresholding the distances between the correspondences.
/// @param src: the input point cloud source
/// @param target: target point cloud
/// @param corresp: the found correspondences (index of query point, index of target point, distance)/the set of initial correspondences given
/// @param maxDist: distance to be used as maximum distance between correspondences. Correspondences with larger distances are rejected.
/// @return the resultant filtered set of remaining correspondences
pcl::Correspondences correspRdist(pcl::PointCloud<pcl::PointNormal>::Ptr src, pcl::PointCloud<pcl::PointNormal>::Ptr target, pcl::Correspondences corresp, const float &maxDist)
{
    LOG("Distance correspondence rejection on " + std::to_string(corresp.size()) + " correspondences" );

    pcl::registration::CorrespondenceRejectorDistance rejector_dist;
    pcl::Correspondences correspRes;
    rejector_dist.setInputSource<pcl::PointNormal>(src);
    rejector_dist.setInputTarget<pcl::PointNormal>(target);
    rejector_dist.setMaximumDistance(maxDist);
    rejector_dist.getRemainingCorrespondences(corresp, correspRes);

    LOG("Kept " + std::to_string(correspRes.size()) + " correspondences (" + std::to_string( (correspRes.size() / corresp.size()) * 100) + "%)");
    return correspRes;
}

/// @author: Gulnur Ungan (Antoine Merlet)
/// @date: 05-01-2018 (06-01-2018)
/// @version 1.0 (1.1)
///
/// @brief Implements a simple correspondence rejection method based on thresholding the distances between the correspondences.
/// @param src: the input point cloud source
/// @param target: target point cloud
/// @param corresp: the found correspondences (index of query point, index of target point, distance)/the set of initial correspondences given
/// @param medFact: the factor for correspondence rejection.
/// @return the resultant filtered set of remaining correspondences
pcl::Correspondences correspRmeddist(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, pcl::Correspondences corresp,const double &medFact)
{
    LOG("Median distance correspondence rejection on " + std::to_string(corresp.size()) + " correspondences" );

    pcl::registration::CorrespondenceRejectorMedianDistance rejector_meddist;
    pcl::Correspondences correspRes;
    rejector_meddist.setInputSource<pcl::PointXYZRGB>(src);
    rejector_meddist.setInputTarget<pcl::PointXYZRGB>(target);
    rejector_meddist.setMedianFactor(medFact);
    rejector_meddist.getRemainingCorrespondences(corresp, correspRes);

    LOG("Kept " + std::to_string(correspRes.size()) + " correspondences (" + std::to_string( (correspRes.size() / (float)corresp.size()) * 100) + "%)");
    return correspRes;
}

/// @author: Gulnur Ungan (Antoine Merlet)
/// @date: 05-01-2018 (06-01-2018)
/// @version 1.0 (1.1)
///
/// @brief Method based on eliminating duplicate match indices in the correspondences.
/// @param corresp:the found correspondences (index of query point, index of target point, distance)/the set of initial correspondences given
/// @return the resultant filtered set of remaining correspondences
pcl::Correspondences correspR121(pcl::Correspondences corresp)
{
    LOG("One to one correspondence rejection on " + std::to_string(corresp.size()) + " correspondences" );
    pcl::registration::CorrespondenceRejectorOneToOne rejector_121;
    pcl::Correspondences correspRes;
    rejector_121.getRemainingCorrespondences(corresp, correspRes);

    LOG("Kept " + std::to_string(correspRes.size()) + " correspondences (" + std::to_string( (correspRes.size() / (float)corresp.size()) * 100) + "%)");
    return correspRes;
}

/// @author: Gulnur Ungan (Antoine Merlet)
/// @date: 05-01-2018 (06-01-2018)
/// @version 1.0 (1.1)
///
/// @brief correspondence rejection using Random Sample Consensus to identify inliers (and reject outliers)
/// @param src: the input point cloud source
/// @param target: target point cloud
/// @param corresp: the found correspondences (index of query point, index of target point, distance)/the set of initial correspondences given
/// @param iter: maximum number if iterations to run
/// @param in_thres: distance threshold in the same dimension as source and target data sets.
/// @return the resultant filtered set of remaining correspondences
pcl::Correspondences correspRransac(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, pcl::Correspondences corresp, const int &iter, const float &in_thres)
{
    LOG("Ransac correspondence rejection on " + std::to_string(corresp.size()) + " correspondences" );

    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> rejector_ransac;
    pcl::Correspondences correspRes;
    rejector_ransac.setInputSource(src);
    rejector_ransac.setInputTarget(target);

    rejector_ransac.setInlierThreshold (in_thres);
    rejector_ransac.setMaxIterations (iter);
    rejector_ransac.getRemainingCorrespondences(corresp,correspRes);

    LOG("Kept " + std::to_string(correspRes.size()) + " correspondences (" + std::to_string( (correspRes.size() / (float)corresp.size()) * 100) + "%)");
    return correspRes;
}

/// @author: Gulnur Ungan (Antoine Merlet)
/// @date: 05-01-2018 (06-01-2018)
/// @version 1.0 (1.1)
///
/// @brief implements a simple correspondence rejection method based on the angle between the normals at correspondent points.
/// @param src: the input point cloud source
/// @param target: target point cloud
/// @param corresp: the found correspondences (index of query point, index of target point, distance)/the set of initial correspondences given
/// @param maxDepthChange:
/// @param smoothSize:
/// @param thres: cosine of the thresholding angle between the normals for rejection
/// @return the resultant filtered set of remaining correspondences
pcl::Correspondences correspRsurfacenorm(pcl::PointCloud<pcl::PointNormal>::Ptr srcPN, pcl::PointCloud<pcl::PointNormal>::Ptr targetPN, pcl::Correspondences corresp, const float &thres)
{
    LOG("Surface Normal correspondence rejection on " + std::to_string(corresp.size()) + " correspondences" );

    pcl::registration::CorrespondenceRejectorSurfaceNormal rejector_norm;
    pcl::Correspondences correspRes;
    pcl::CorrespondencesPtr corresp_tmp (new pcl::Correspondences(corresp));
    rejector_norm.setThreshold(thres);
    rejector_norm.initializeDataContainer<pcl::PointNormal,pcl::PointNormal>();
    rejector_norm.setInputSource<pcl::PointNormal>(srcPN);
    rejector_norm.setInputTarget<pcl::PointNormal>(targetPN);
    rejector_norm.setInputNormals<pcl::PointNormal, pcl::PointNormal>(srcPN);
    rejector_norm.setTargetNormals<pcl::PointNormal, pcl::PointNormal>(targetPN);
    rejector_norm.setInputCorrespondences(corresp_tmp);
    rejector_norm.getCorrespondences(correspRes);

    LOG("Kept " + std::to_string(correspRes.size()) + " correspondences (" + std::to_string( (correspRes.size() / (float)corresp.size()) * 100) + "%)");
    return correspRes;
}

/// @author: Gulnur Ungan (Antoine Merlet)
/// @date: 05-01-2018 (06-01-2018)
/// @version 1.0 (1.1)
///
/// @brief implements a simple correspondence rejection method to reject boundary points
/// @param src: the input point cloud source
/// @param target: target point cloud
/// @param corresp: the found correspondences (index of query point, index of target point, distance)/the set of initial correspondences given
/// @return the resultant filtered set of remaining correspondences
pcl::Correspondences correspRboudary(pcl::PointCloud<pcl::PointNormal>::Ptr src, pcl::PointCloud<pcl::PointNormal>::Ptr target, pcl::Correspondences corresp)
{

    LOG("Boundary correspondence rejection on " + std::to_string(corresp.size()) + " correspondences" );
    src->width = 640;
    src->height = 480;
    src->points.resize(src->width * src->height);
    target->width = 640;
    target->height = 480;
    target->points.resize(target->width * target->height);
    pcl::registration::CorrespondenceRejectionOrganizedBoundary rejector_bound;
    pcl::Correspondences correspRes;
    rejector_bound.setInputSource<pcl::PointNormal>(src);
    rejector_bound.setInputTarget<pcl::PointNormal>(target);
    rejector_bound.getRemainingCorrespondences(corresp, correspRes);

    LOG("Kept " + std::to_string(correspRes.size()) + " correspondences (" + std::to_string( (correspRes.size() / (float)corresp.size()) * 100) + "%)");
    return correspRes;
}





/// @author: Antoine Merlet
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief register a pair of point clouds
/// @param src: the source point cloud
/// @param target: the target point cloud
/// @param icp mode: 1: LLS point to plane, 2: SVD, 3: LM point to point, 4: LM point to plane
/// @param iter: the number of iterations for ransac rejection
/// @param maxDist: the maximum between two points to be said corresponding
/// @param medFact: the median factor for median distance rejection
/// @param thres: the threshold and value for rejaction by surface norme
/// @param in_thres: the inlier threshold value for ransac rejaction
/// @return the transformation matrix
Eigen::Matrix4d pairRegister(pcl::PointCloud<pcl::PointXYZRGB>::Ptr srcPN, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPN, pcl::Correspondences corresp, const int &icpMode, const int &maxiter, const int &iter_simi, const float &mser, const float &msea)
{
    boost::shared_ptr< pcl::registration::TransformationEstimation< pcl::PointXYZRGB, pcl::PointXYZRGB, double > > estPtr;
    switch(icpMode) {
    case 1:
        //estPtr.reset ( new pcl::registration::TransformationEstimationPointToPlaneLLS < pcl::PointXYZRGB, pcl::PointXYZRGB, double  > () );
        LOG(" ---- Running PointToPlaneLLS");
        break;
    case 2:
        estPtr.reset ( new pcl::registration::TransformationEstimationSVD < pcl::PointXYZRGB, pcl::PointXYZRGB, double  > () );
        LOG(" ---- Running SVD");
        break;
    case 3:
        estPtr.reset ( new pcl::registration::TransformationEstimationLM < pcl::PointXYZRGB, pcl::PointXYZRGB, double  > () );
        LOG(" ---- Running PointToPointLM");
        break;
    case 4:
        //estPtr.reset ( new pcl::registration::TransformationEstimationPointToPlane < pcl::PointXYZRGB, pcl::PointXYZRGB , double > () );
        LOG(" ---- Running PointToPlaneLM");
        break;
    default:
        break;
    }

    Eigen::Matrix4d final_transform (Eigen::Matrix4d::Identity()), transform (Eigen::Matrix4d::Identity ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);

    int iterations = 0;
    pcl::registration::DefaultConvergenceCriteria<double> converged(iterations, transform, corresp);
    converged.setMaximumIterations(maxiter);

    if (iter_simi != 0)
        converged.setMaximumIterationsSimilarTransforms(iter_simi);
    if (mser != 0)
        converged.setRelativeMSE(mser);
    if (iter_simi != 0)
        converged.setAbsoluteMSE(msea);

    do{

        pcl::transformPointCloud(*srcPN, *output, final_transform);
        estPtr->estimateRigidTransformation(*output,*targetPN,transform);
        std::cout << transform << endl<< endl<< endl;
        final_transform = transform * final_transform;
        ++iterations;

    }while(iterations < maxiter);

    return final_transform;
}

/// @author: Antoine Merlet
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief register a pair of point clouds using point to surface LLS
/// @param src: the source point cloud
/// @param target: the target point cloud
/// @param iter: the number of iterations for ransac rejection
/// @param maxDist: the maximum between two points to be said corresponding
/// @param medFact: the median factor for median distance rejection
/// @param thres: the threshold and value for rejaction by surface norme
/// @param in_thres: the inlier threshold value for ransac rejaction
/// @return the transformation matrix
Eigen::Matrix4d icpLLSp2s(pcl::PointCloud<pcl::PointNormal>::Ptr srcPN, pcl::PointCloud<pcl::PointNormal>::Ptr targetPN, pcl::Correspondences corresp, const int &maxiter, const int &iter_simi, const float &mser, const float &msea)
{
    pcl::registration::TransformationEstimationPointToPlaneLLS< pcl::PointNormal, pcl::PointNormal, double > teLLS;
    Eigen::Matrix4d final_transform (Eigen::Matrix4d::Identity()), transform (Eigen::Matrix4d::Identity ());
    pcl::PointCloud<pcl::PointNormal>::Ptr output (new pcl::PointCloud<pcl::PointNormal>);
    *output = *srcPN;

    int iterations = 0;
    pcl::registration::DefaultConvergenceCriteria<double> converged(iterations, transform, corresp);
    converged.setMaximumIterations(maxiter);
    if (iter_simi != 0)
        converged.setMaximumIterationsSimilarTransforms(iter_simi);
    if (mser != 0)
        converged.setRelativeMSE(mser);
    if (iter_simi != 0)
        converged.setAbsoluteMSE(msea);

    do{
        //        *corresp = fullCorresp(output, target, iter, radius, medFac, thres, in_thres);
        //        teLLS.estimateRigidTransformation(*output,*target,*corresp,transform);
        //        final_transform = transform * final_transform;
        //        pcl::transformPointCloudWithNormals (*src, *output, final_transform.cast<float> ());
        //        ++iterations;
    }while(!converged);
    return final_transform;
}


/// @author: Antoine Merlet
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief register a pair of point clouds using SVD
/// @param src: the source point cloud
/// @param target: the target point cloud
/// @param iter: the number of iterations for ransac rejection
/// @param maxDist: the maximum between two points to be said corresponding
/// @param medFact: the median factor for median distance rejection
/// @param thres: the threshold and value for rejaction by surface norme
/// @param in_thres: the inlier threshold value for ransac rejaction
/// @return the transformation matrix
Eigen::Matrix4d icpSVD(pcl::PointCloud<pcl::PointNormal>::Ptr srcPN, pcl::PointCloud<pcl::PointNormal>::Ptr targetPN, pcl::Correspondences corresp, const int &maxiter, const int &iter_simi, const float &mser, const float &msea)
{
    pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal, double> teSVD;
    pcl::registration::TransformationEstimationSVD<pcl::PointNormal,pcl::PointNormal, double>::Matrix4 transform;
    Eigen::Matrix4d final_transform (Eigen::Matrix4d::Identity());
    pcl::PointCloud<pcl::PointNormal>::Ptr output (new pcl::PointCloud<pcl::PointNormal>);
    int iterations = 0;
    pcl::registration::DefaultConvergenceCriteria<double> converged(iterations, transform, corresp);
    converged.setMaximumIterations(maxiter);
    if (iter_simi != 0)
        converged.setMaximumIterationsSimilarTransforms(iter_simi);
    if (mser != 0)
        converged.setRelativeMSE(mser);
    if (iter_simi != 0)
        converged.setAbsoluteMSE(msea);
    LOG(" ---- Running SVD ICP on src: " + std::to_string(srcPN->size()));

    do{
        pcl::transformPointCloudWithNormals(*srcPN, *output, final_transform);

        teSVD.estimateRigidTransformation(*output,*targetPN,corresp,transform);
        std::cout << transform << endl;
        final_transform = transform * final_transform;
        ++iterations;
    }while(!converged.hasConverged());

    std::cout << iterations << endl;
    return final_transform;
}


/// @author: Antoine Merlet
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief register a pair of point clouds using point to surface Levenberg Marquardt point to point optimization
/// @param src: the source point cloud
/// @param target: the target point cloud
/// @param iter: the number of iterations for ransac rejection
/// @param maxDist: the maximum between two points to be said corresponding
/// @param medFact: the median factor for median distance rejection
/// @param thres: the threshold and value for rejaction by surface norme
/// @param in_thres: the inlier threshold value for ransac rejaction
/// @return the transformation matrix
Eigen::Matrix4d icpLMp2p(pcl::PointCloud<pcl::PointNormal>::Ptr srcPN, pcl::PointCloud<pcl::PointNormal>::Ptr targetPN, pcl::Correspondences corresp, const int &maxiter, const int &iter_simi, const float &mser, const float &msea)
{
    pcl::registration::TransformationEstimationLM< pcl::PointNormal, pcl::PointNormal, double > teLM;
    Eigen::Matrix4d final_transform (Eigen::Matrix4d::Identity()), transform (Eigen::Matrix4d::Identity ());
    pcl::PointCloud<pcl::PointNormal>::Ptr output (new pcl::PointCloud<pcl::PointNormal>);
    *output = *srcPN;

    int iterations = 0;
    pcl::registration::DefaultConvergenceCriteria<double> converged(iterations, transform, corresp);
    converged.setMaximumIterations(maxiter);
    if (iter_simi != 0)
        converged.setMaximumIterationsSimilarTransforms(iter_simi);
    if (mser != 0)
        converged.setRelativeMSE(mser);
    if (iter_simi != 0)
        converged.setAbsoluteMSE(msea);

    do{
        //        *corresp = fullCorresp(output, target, iter, maxDist, medFac, thres, in_thres);
        //        teLM.estimateRigidTransformation(*output,*target,*corresp,transform);
        //        final_transform = transform * final_transform;
        //        pcl::transformPointCloudWithNormals (*src, *output, final_transform.cast<float> ());
        //        ++iterations;
    }while(!converged);
    return final_transform;
}


/// @author: Antoine Merlet
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief register a pair of point clouds using point to surface Levenberg Marquardt point to surface optimization
/// @param src: the source point cloud
/// @param target: the target point cloud
/// @param iter: the number of iterations for ransac rejection
/// @param maxDist: the maximum between two points to be said corresponding
/// @param medFact: the median factor for median distance rejection
/// @param thres: the threshold and value for rejaction by surface norme
/// @param in_thres: the inlier threshold value for ransac rejaction
/// @return the transformation matrix
Eigen::Matrix4d icpLMp2s(pcl::PointCloud<pcl::PointNormal>::Ptr srcPN, pcl::PointCloud<pcl::PointNormal>::Ptr targetPN, pcl::Correspondences corresp, const int &maxiter, const int &iter_simi, const float &mser, const float &msea)
{
    pcl::registration::TransformationEstimationPointToPlane< pcl::PointNormal, pcl::PointNormal, double > teLM;
    Eigen::Matrix4d final_transform (Eigen::Matrix4d::Identity()), transform (Eigen::Matrix4d::Identity ());
    pcl::PointCloud<pcl::PointNormal>::Ptr output (new pcl::PointCloud<pcl::PointNormal>);
    *output = *srcPN;

    int iterations = 0;
    pcl::registration::DefaultConvergenceCriteria<double> converged(iterations, transform, corresp);
    converged.setMaximumIterations(maxiter);
    if (iter_simi != 0)
        converged.setMaximumIterationsSimilarTransforms(iter_simi);
    if (mser != 0)
        converged.setRelativeMSE(mser);
    if (iter_simi != 0)
        converged.setAbsoluteMSE(msea);

    do{
        //        *corresp = fullCorresp(output, target, iter, maxDist, medFac, thres, in_thres);
        //        teLM.estimateRigidTransformation(*output,*target,*corresp,transform);
        //        final_transform = transform * final_transform;
        //        pcl::transformPointCloudWithNormals (*src, *output, final_transform.cast<float> ());
        //        ++iterations;
    }while(!converged);
    return final_transform;
}
}

