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
#include <pcl/filters/extract_indices.h>

namespace Core {

/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Function used to downsample the given Point Cloud
/// @param pc The Point Cloud to downsample
/// @param x y z The size of the filter
/// @return The downsampled Point Cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const float &x, const float &y, const float &z)
{
    LOG("VoxelGrid on " + std::to_string(pc->size()) + "points ...");
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    grid.setLeafSize (x, y, z);
    grid.setInputCloud (pc);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);
    src->width = 640;
    src->height = 480;
    src->points.resize(src->width * src->height);
    grid.filter (*src);
    LOG("Downsampling Done. Now " + std::to_string(src->size()) + " points");
    return src;
}

/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Implementation of a fast bilateral filter for smoothing depth information in organized point clouds
/// @param cloud_in: The Point Cloud to downsample
/// @param sigmaR: The standard deviation of the Gaussian for the intensity difference
/// @param sigmaS: The size of the Gaussian bilateral filter window to use
/// @return The final point cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr bilateralFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const float &sigmaR, const float &sigmaS)
{
    if (cloud_in->isOrganized())
    {
        LOG("Bilateral Filtering on " + std::to_string(cloud_in->size()) + "points ...");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_out->width = 640;
        cloud_out->height = 480;
        cloud_out->points.resize(cloud_out->width * cloud_out->height);
        pcl::FastBilateralFilter<pcl::PointXYZRGB> fbf;
        fbf.setSigmaS(sigmaS);
        fbf.setSigmaR(sigmaR);
        fbf.setInputCloud(cloud_in);
        fbf.filter(*cloud_out);
        LOG("Downsampling Done. Now " + std::to_string(cloud_out->size()) + " points " );
        return cloud_out;

    }
    else
    {
        LOG(" Point Cloud non organized. Skipping Bilateral Filtering." );
        return cloud_in;
    }
}

/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Takes in a colored organized point cloud, that might contain non zero values for the depth information.
/// @param cloud_in: The Point Cloud to downsample
/// @param sigmaC: the new value to be set. It is called sigmacolor.
/// @param sigmaD: the new value to be set. It is called sigmadepth.
/// @return An upsampled version of this cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr bilateralupsamplerRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const float &sigmaC, const float &sigmaD)
{
    LOG("Bilateral upsampling on " + std::to_string(cloud_in->size()) + "points ...");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out;

    pcl::BilateralUpsampling<pcl::PointXYZRGB, pcl::PointXYZRGB> bus;
    bus.setSigmaColor(sigmaC);
    bus.setSigmaDepth(sigmaD);
    bus.setInputCloud(cloud_in);
    bus.process(*cloud_out);

    LOG("Upsampling Done. Now " + std::to_string(cloud_out->size()) + " points");
    return cloud_out;
}

/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Implementation of the median filter.
/// @param cloud_in: The Point Cloud to downsample
/// @param windowSize: Set the window size of the filter.
/// @param maxMovement: Maximum value a dexel is allowed to move during filtering
/// @return Resultant point cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr medianFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const int &windowSize, const float &maxMovement)
{
    if (cloud_in->isOrganized())
    {
    LOG("Median filtering on " + std::to_string(cloud_in->size()) + "points ...");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_out->width = 640;
    cloud_out->height = 480;
    cloud_out->points.resize(cloud_out->width * cloud_out->height);
    pcl::MedianFilter<pcl::PointXYZRGB> mf;
    mf.setWindowSize(windowSize);
    mf.setMaxAllowedMovement(maxMovement);
    mf.setInputCloud(cloud_in);
    mf.filter(*cloud_out);

    LOG("Downsampling Done. Now " + std::to_string(cloud_out->size()) + " points");
    return cloud_out;
    }
    else
    {
        LOG(" Point Cloud non organized. Skipping Median Filtering." );
        return cloud_in;
    }
}

/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Random sampling with uniform probability. Based on an article. Results: http://www.ittc.ku.edu/~jsv/Papers/Vit84.sampling.pdf
/// @param cloud_in: The Point Cloud to downsample
/// @param order: downsampling ratio
/// @return Resultant point cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr randomSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const int &order)
{
    LOG("Random sampling on " + std::to_string(cloud_in->size()) + "points ...");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_out->width = 640;
    cloud_out->height = 480;
    cloud_out->points.resize(cloud_out->width * cloud_out->height);
    pcl::RandomSample<pcl::PointXYZRGB> randsample;
    randsample.setSample(cloud_in->size() / order);
    randsample.setInputCloud(cloud_in);
    randsample.filter(*cloud_out);

    LOG("Downsampling Done. Now " + std::to_string(cloud_out->size()) + " points");
    return cloud_out;
}

/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief It samples the input point cloud in the space of normal directions computed at every point.
/// @param cloud_in: The Point Cloud to downsample
/// @param order: downsampling ratio
/// @param nbBins: Number of bins.
/// @param maxDepthChange: the depth change threshold for computing object borders based on depth changes
/// @param smoothSize: smooth size factor which influences the size of the area used to smooth normals (depth dependent if useDepthDependentSmoothing is true)
/// @return Resultant point cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr normalSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const unsigned int &order, const unsigned int &nbBins, const float &radius)
{
    LOG("Normal  sampling on " + std::to_string(cloud_in->size()) + "points ...");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_out->width = 640;
    cloud_out->height = 480;
    cloud_out->points.resize(cloud_out->width * cloud_out->height);
    pcl::NormalSpaceSampling<pcl::PointXYZRGB, pcl::PointNormal> normsample;
    normsample.setBins (nbBins, nbBins, nbBins);
    normsample.setSample(cloud_in->size() / order);
    normsample.setInputCloud(cloud_in);
    normsample.setNormals (getNormalPoints(cloud_in, radius));
    normsample.filter(*cloud_out);

    LOG("Downsampling Done. Now " + std::to_string(cloud_out->size()) + " points");
    return cloud_out;
}

/// @author: Gulnur Ungan
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief It selects the points such that the resulting cloud is as stable as possible for being registered (against a copy of itself) with ICP.
/// @param cloud_in: The Point Cloud to downsample
/// @param order: downsampling ratio
/// @param maxDepthChange: the depth change threshold for computing object borders based on depth changes
/// @param smoothSize: smooth size factor which influences the size of the area used to smooth normals (depth dependent if useDepthDependentSmoothing is true)
/// @return Resultant point cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr covarianceSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const unsigned int &order, const float &radius)
{
    LOG("Covariance  sampling on " + std::to_string(cloud_in->size()) + "points ...");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_in));
    cloud_out->width = 640;
    cloud_out->height = 480;
    cloud_out->points.resize(cloud_out->width * cloud_out->height);

    pcl::CovarianceSampling< pcl::PointNormal, pcl::PointNormal> covSampling;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals = getNormalPoints(cloud_in, radius);
    pcl::PointCloud<pcl::PointNormal>::Ptr out (new pcl::PointCloud<pcl::PointNormal>);
    covSampling.setNumberOfSamples(normals->size() / order);
    covSampling.setKeepOrganized (true);
    covSampling.setInputCloud(normals);
    covSampling.setNormals(normals);

    covSampling.filter (*out);
    // good size
//    LOG("zeryhthd " + std::to_string(out->size()) + " points");

//    pcl::IndicesPtr indices (new std::vector<int> ());
//    indices = covSampling.getIndices();

//    LOG("indice 255 " + std::to_string(indices->at(255)) + " points");
//    pcl::ExtractIndices<pcl::PointXYZRGB> eifilter (true); // Initializing with true will allow us to extract the removed indices
//    eifilter.setInputCloud (cloud_in);
//    eifilter.setIndices(indices);
//    eifilter.setKeepOrganized(true);
//    LOG("Downsampling Done. Now " + std::to_string(cloud_out->size()) + " points");
//    eifilter.filter (*cloud_out);
//    LOG("Dzetg dw " + std::to_string(cloud_out->size()) + " points");
    return cloud_out;
}

}
