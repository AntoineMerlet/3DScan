#ifndef DATABASE_H
#define DATABASE_H
#include <string>
#include <vector>
#include <QStringList>

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>

class DataBase
{
public:
    DataBase();
    ~DataBase();

private:
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> rawPCs;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> registeredPCs;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> filteredPCs;
    std::vector<pcl::PolygonMesh::Ptr> meshedPC;
};

#endif // DATABASE_H
