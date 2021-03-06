#ifndef IOTOOLS_H
#define IOTOOLS_H

#include <string>
#include <vector>
#include <QStringList>

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
namespace IO {

pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPCD(const std::string &);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPLY(const std::string &);

pcl::PolygonMesh::Ptr loadSTL(const std::string &);
pcl::PolygonMesh::Ptr loadVTK(const std::string &);

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> loadPCD(const std::vector<std::string> &);
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> loadPLY(const std::vector<std::string> &);
std::vector<pcl::PolygonMesh::Ptr> loadSTL(const std::vector<std::string> &);
std::vector<pcl::PolygonMesh::Ptr> loadVTK(const std::vector<std::string> &);

bool savePCD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &, const std::string &);
bool savePLY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &, const std::string &);
bool saveSTL(pcl::PolygonMesh::Ptr &, const std::string &);
bool saveVTK(pcl::PolygonMesh::Ptr &, const std::string &);

bool savePCD(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &, const std::string &, const std::vector<std::string> &);
bool savePLY(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &, const std::string &, const std::vector<std::string> &);
bool saveSTL(std::vector<pcl::PolygonMesh::Ptr> &, const std::string &, const std::vector<std::string> &);
bool saveVTK(std::vector<pcl::PolygonMesh::Ptr> &, const std::string &, const std::vector<std::string> &);

}

#endif // IOTOOLS_H
