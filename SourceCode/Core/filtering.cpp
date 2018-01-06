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
/// @brief Function used to downsample the given Point Cloud
/// @param pc The Point Cloud to downsample
/// @param x y z The size of the filter
/// @return The downsampled Point Cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const float &x, const float &y, const float &z){
    LOG("Downsampling ...");
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    grid.setLeafSize (x, y, z);
    grid.setInputCloud (pc);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);
    grid.filter (*src);
    LOG("Downsampling Done.");
    return src;
}

/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Implementation of a fast bilateral filter for smoothing depth information in organized point clouds
/// @param cloud_in :The Point Cloud to downsample
/// @param sigmaR: The standard deviation of the Gaussian for the intensity difference
/// @param sigmaS: The size of the Gaussian bilateral filter window to use
/// @return The final point cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr bilateralFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const float &sigmaR, const float &sigmaS)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::FastBilateralFilter<pcl::PointXYZRGB> fbf;
    fbf.setSigmaS(sigmaS);
    fbf.setSigmaR(sigmaR);
    fbf.setInputCloud(cloud_in);

    fbf.filter(*cloud_out);
    return cloud_out;
}

/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Takes in a colored organized point cloud, that might contain non zero values for the depth information.
/// @param cloud_in : The Point Cloud to downsample
/// @param sigmaC: the new value to be set. It is called sigmacolor.
/// @param sigmaD: the new value to be set. It is called sigmadepth.
/// @return An upsampled version of this cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr bilateralupsamplerRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const float &sigmaC, const float &sigmaD)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out;

    pcl::BilateralUpsampling<pcl::PointXYZRGB, pcl::PointXYZRGB> bus;
    bus.setSigmaColor(sigmaC);
    bus.setSigmaDepth(sigmaD);
    bus.setInputCloud(cloud_in);

    bus.process(*cloud_out);
    return cloud_out;
}

/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief:Implementation of the median filter.
/// @param cloud_in : The Point Cloud to downsample
/// @param windowSize: Set the window size of the filter.
/// @param maxMovement: Maximum value a dexel is allowed to move during filtering
/// @return Resultant point cloud
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

/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief :Random sampling with uniform probability. Based on an article. Results: http://www.ittc.ku.edu/~jsv/Papers/Vit84.sampling.pdf
/// @param cloud_in : The Point Cloud to downsample
/// @param order: downsampling ratio
/// @return Resultant point cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const unsigned int &order)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::RandomSample<pcl::PointXYZRGB> randsample;
    randsample.setSample(cloud_in->size() / order);
    randsample.setInputCloud(cloud_in);

    randsample.filter(*cloud_out);
    return cloud_out;
}

/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief : It samples the input point cloud in the space of normal directions computed at every point.
/// @param cloud_in : The Point Cloud to downsample
/// @param order: downsampling ratio
/// @param nbBins: Number of bins.
/// @param maxDepthChange: the depth change threshold for computing object borders based on depth changes
/// @param smoothSize: smooth size factor which influences the size of the area used to smooth normals (depth dependent if useDepthDependentSmoothing is true)
/// @return Resultant point cloud
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

/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief : It selects the points such that the resulting cloud is as stable as possible for being registered (against a copy of itself) with ICP.
/// @param cloud_in : The Point Cloud to downsample
/// @param order: downsampling ratio
/// @param maxDepthChange: the depth change threshold for computing object borders based on depth changes
/// @param smoothSize: smooth size factor which influences the size of the area used to smooth normals (depth dependent if useDepthDependentSmoothing is true)
/// @return Resultant point cloud
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





}
