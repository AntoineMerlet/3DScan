#include "filtering.h"
#include "Core/mathtools.h"
#include "pcl/filters/fast_bilateral.h"
#include <pcl/filters/normal_space.h>
#include "pcl/registration/icp.h"
#include <pcl/registration/ia_ransac.h>
#include <pcl/surface/bilateral_upsampling.h>
#include <pcl/filters/median_filter.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/covariance_sampling.h>

namespace Core {

/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Function used to load the .PCD file given by path and returning the loaded Point Cloud.
/// @param pc The Point Cloud to downsample
/// @return The downsampled Point Cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc){
    LOG("Downsampling ...");
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    grid.setLeafSize (0.01, 0.01, 0.01);
    grid.setInputCloud (pc);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);
    grid.filter (*src);
    LOG("Downsampling Done.");
    return src;
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


/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Function used to load the .PCD file given by path and returning the loaded Point Cloud.
/// @param cloud_in The Point Cloud to downsample
/// @param
/// @return The downsampled Point Cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr bilateralFitler(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const float &sigmaR, const float &sigmaS)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::FastBilateralFilter<pcl::PointXYZRGB> fbf;
    fbf.setSigmaS(sigmaS);
    fbf.setSigmaR(sigmaR);
    fbf.setInputCloud(cloud_in);

    fbf.filter(*cloud_out);
    return cloud_out;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr bilateralupsamblerRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const float &sigmaC, const float &sigmaD)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out;

    pcl::BilateralUpsampling<pcl::PointXYZRGB, pcl::PointXYZRGB> bus;
    bus.setSigmaColor(sigmaC);
    bus.setSigmaDepth(sigmaD);
    bus.setInputCloud(cloud_in);

    bus.process(*cloud_out);
    return cloud_out;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr medianFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const int &windowSize, const float &maxMovement)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::MedianFilter<pcl::PointXYZRGB> mf;
    mf.setWindowSize(windowSize);
    mf.setMaxAllowedMovement(maxMovement);
    mf.setInputCloud(cloud_in);

    mf.filter(*cloud_out);
    return cloud_out;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const unsigned int &order)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::RandomSample<pcl::PointXYZRGB> randsample;
    randsample.setSample(cloud_in->size() / order);
    randsample.setInputCloud(cloud_in);

    randsample.filter(*cloud_out);
    return cloud_out;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr normalSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const unsigned int &order, const unsigned int &nbBins, const float &maxDepthChange, const float &smoothSize)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::NormalSpaceSampling<pcl::PointXYZRGB, pcl::PointNormal> normsample;
    normsample.setBins (nbBins, nbBins, nbBins);
    normsample.setSample(cloud_in->size() / order);
    normsample.setInputCloud(cloud_in);
    normsample.setNormals (getNormalPoints(cloud_in, maxDepthChange, smoothSize));
    normsample.filter(*cloud_out);
    return cloud_out;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr covarianceSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const unsigned int &order, const float &maxDepthChange, const float &smoothSize)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_in_const(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_in));

    pcl::CovarianceSampling<pcl::PointXYZRGB, pcl::PointNormal> covSampling;
    covSampling.setNumberOfSamples(cloud_in->size() / order);;
    covSampling.setKeepOrganized (true);
    covSampling.setInputCloud(cloud_in_const);
    covSampling.setNormals(getNormalPoints(cloud_in, maxDepthChange, smoothSize));
    covSampling.filter(*cloud_out);
    return cloud_out;
}

pcl::PointCloud<pcl::PointNormal>::Ptr getNormalPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const float &maxDepthChange, const float &smoothSize )
{
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>normest;

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    normest.setNormalEstimationMethod(normest.AVERAGE_DEPTH_CHANGE);
    normest.setMaxDepthChangeFactor(maxDepthChange);
    normest.setDepthDependentSmoothing(true);
    normest.setNormalSmoothingSize(smoothSize);
    normest.setInputCloud(cloud_in);

    normest.compute(*normals);


    return normal2PointNormal(cloud_in,normals);
}
}
