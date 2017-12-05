#ifndef IOTOOLS_H
#define IOTOOLS_H

#include <string>
#include <vector>
namespace IO {

pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPCD(std::string);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPLY(std::string);
pcl::PolygonMesh::Ptr loadSTL(std::string);
pcl::PolygonMesh::Ptr loadVTK(std::string);

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> loadPCD(QStringList);
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> loadPLY(QStringList);
std::vector<pcl::PolygonMesh::Ptr> loadSTL(QStringList);
std::vector<pcl::PolygonMesh::Ptr> loadVTK(QStringList);

bool savePCD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::string);
bool savePLY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::string);
bool saveSTL(pcl::PolygonMesh::Ptr, std::string);
bool saveVTK(pcl::PolygonMesh::Ptr, std::string);

bool savePCD(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>, std::string);
bool savePLY(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>, std::string);
bool saveSTL(vector<pcl::PolygonMesh::Ptr>, std::string);
bool saveVTK(vector<pcl::PolygonMesh::Ptr>, std::string);

}

#endif // IOTOOLS_H
