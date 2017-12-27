#ifndef DATABASE_H
#define DATABASE_H
#include <string>
#include <vector>
#include <list>

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>

class DataBase
{
public:
    DataBase(){}
    ~DataBase(){}

    // Getters
    // All PC
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>  getRawPCs();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getRegisteredPCs();
    std::vector<pcl::PolygonMesh::Ptr> getMeshedPCs();
    // Only PC of index specified in list
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>  getRawPCs(std::list<int>);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getRegisteredPCs(std::list<int>);
    std::vector<pcl::PolygonMesh::Ptr> getMeshedPCs(std::list<int>);
    // Only one PC at the given index
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  getRawPC(const int &);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getRegisteredPC(const int &);
    pcl::PolygonMesh::Ptr getMeshedPC(const int &);


    // Setters
    // Append a new PC
    bool addRawPC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &);
    bool addRegisteredPC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &);
    bool addMeshedPC(const pcl::PolygonMesh::Ptr &);
    // Replace the PC at given location with PC given as parameter
    bool replaceRawPC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &, const int &);
    bool replaceRegisteredPC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &, const int &);
    bool replaceMeshedPC(const pcl::PolygonMesh::Ptr &, const int &);
    // Delte the PC at given location
    bool removeRawPC(const int &);
    bool removeRegisteredPC(const int &);
    bool removeMeshedPC(const int &);

private:
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> rawPCs;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> registeredPCs;
    std::vector<pcl::PolygonMesh::Ptr> meshedPCs;
};

#endif // DATABASE_H
