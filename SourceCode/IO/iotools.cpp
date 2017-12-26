#include "iotools.h"
#include <exception>

namespace IO {

/// @author: Antoine Merlet
/// @date: 15-12-2017
/// @version 1.0
///
/// @brief Function used to load the .PCD file given by path and returning the loaded point cloud.
/// @param path The path of the file to load.
/// @return The loaded point cloud in XYZ coordinates and RGB colors, as pointer. Or nullptr if errors while loading.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPCD(const std::string &path)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrPC (new pcl::PointCloud<pcl::PointXYZRGB>());
    try
    {
        pcl::io::loadPCDFile(path, *ptrPC);
        // add emit info to console here
        return ptrPC;
    }
    catch (const std::exception &e)
    {
        // add emit info to console here e.what();
        return nullptr;
    }
}

/// @author: Antoine Merlet
/// @date: 15-12-2017
/// @version 1.0
///
/// @brief Function used to load the .PLY file given by path and returning the loaded point cloud.
/// @param path The path of the file to load.
/// @return The loaded point cloud in XYZ coordinates and RGB colors, as pointer. Or nullptr if errors while loading.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPLY(const std::string &path)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrPC (new pcl::PointCloud<pcl::PointXYZRGB>());
    try
    {
        pcl::io::loadPLYFile(path, *ptrPC);
        // add emit info to console here
        return ptrPC;
    }
    catch (const std::exception &e)
    {
        // add emit info to console here e.what();
        return nullptr;
    }
}

/// @author: Antoine Merlet
/// @date: 15-12-2017
/// @version 1.0
///
/// @brief Function used to load the .STL file given by path and returning the loaded mesh.
/// @param path The path of the file to load.
/// @return The loaded mesh pointer. Or nullptr if errors while loading.
pcl::PolygonMesh::Ptr loadSTL(const std::string &path)
{
    pcl::PolygonMesh::Ptr ptrMesh (new pcl::PolygonMesh());
    try
    {
        pcl::io::loadPolygonFileSTL(path, *ptrMesh);
        // add emit info to console here
        return ptrMesh;
    }
    catch (const std::exception &e)
    {
        // add emit info to console here e.what();
        return nullptr;
    }
}

/// @author: Antoine Merlet
/// @date: 15-12-2017
/// @version 1.0
///
/// @brief Function used to load the .VTK file given by path and returning the loaded mesh.
/// @param path The path of the file to load.
/// @return The loaded mesh pointer. Or nullptr if errors while loading.
pcl::PolygonMesh::Ptr loadVTK(const std::string &path)
{
    pcl::PolygonMesh::Ptr ptrMesh (new pcl::PolygonMesh());
    try
    {
        pcl::io::loadPolygonFileVTK(path, *ptrMesh);
        // add emit info to console here
        return ptrMesh;
    }
    catch (const std::exception &e)
    {
        // add emit info to console here e.what();
        return nullptr;
    }
}


std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> loadPCD(const std::vector<std::string> &vecPaths)
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vecPC;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrPC;
    for (std::vector<std::string>::const_iterator   it = vecPaths.begin() ; it != vecPaths.end(); ++it)
    {
        ptrPC = loadPCD(*it);
        if (ptrPC != nullptr)
            vecPC.push_back(ptrPC);
    }
    return vecPC;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> loadPLY(const std::vector<std::string> &vecPaths)
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vecPC;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrPC;
    for (std::vector<std::string>::const_iterator   it = vecPaths.begin() ; it != vecPaths.end(); ++it)
    {
        ptrPC = loadPLY(*it);
        if (ptrPC != nullptr)
            vecPC.push_back(ptrPC);
    }
    return vecPC;
}

std::vector<pcl::PolygonMesh::Ptr> loadSTL(const std::vector<std::string> &vecPaths)
{
    std::vector<pcl::PolygonMesh::Ptr> vecMesh;
    pcl::PolygonMesh::Ptr ptrMesh;
    for (std::vector<std::string>::const_iterator  it = vecPaths.begin() ; it != vecPaths.end(); ++it)
    {
        ptrMesh = loadSTL(*it);
        if (ptrMesh != nullptr)
            vecMesh.push_back(ptrMesh);
    }
    return vecMesh;
}

std::vector<pcl::PolygonMesh::Ptr> loadVTK(const std::vector<std::string> &vecPaths)
{
    std::vector<pcl::PolygonMesh::Ptr> vecMesh;
    pcl::PolygonMesh::Ptr ptrMesh;
    for (std::vector<std::string>::const_iterator  it = vecPaths.begin() ; it != vecPaths.end(); ++it)
    {
        ptrMesh = loadVTK(*it);
        if (ptrMesh != nullptr)
            vecMesh.push_back(ptrMesh);
    }
    return vecMesh;
}



bool savePCD(pcl::PointCloud<const pcl::PointXYZRGB>::Ptr &ptrPC,const std::string &path)
{
    try
    {
        pcl::io::savePCDFile(path, *ptrPC);
        // add emit info to console here
        return true;
    }
    catch (const std::exception &e)
    {
        // add emit info to console here e.what();
        return false;
    }
}

bool savePLY(pcl::PointCloud<const pcl::PointXYZRGB>::Ptr &ptrPC,const std::string &path)
{
    try
    {
        pcl::io::savePLYFile(path, *ptrPC);
        // add emit info to console here
        return true;
    }
    catch (const std::exception &e)
    {
        // add emit info to console here e.what();
        return false;
    }
}

bool saveSTL(const pcl::PolygonMesh::Ptr &ptrMesh,const std::string &path)
{
    try
    {
        pcl::io::savePolygonFileSTL(path, *ptrMesh);
        // add emit info to console here
        return true;
    }
    catch (const std::exception &e)
    {
        // add emit info to console here e.what();
        return false;
    }
}

bool saveVTK(const pcl::PolygonMesh::Ptr &ptrMesh,const std::string &path)
{
    try
    {
        pcl::io::savePolygonFileVTK(path, *ptrMesh);
        // add emit info to console here
        return true;
    }
    catch (const std::exception &e)
    {
        // add emit info to console here e.what();
        return false;
    }
}


int savePCD(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &vecPC,const std::string &dir,const std::vector<std::string> vecPaths)
{
    int fails = 0;
    std::vector<std::string>::const_iterator  itPath = vecPaths.begin();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::const_iterator  itPC = vecPC.begin();
    for (itPath, itPC; itPath != vecPaths.end(), itPC != vecPC.end(); ++itPath, ++itPC)
        if (!savePCD(*itPC, dir + "/" + *itPath))
            fails++;
    return fails;
}

int savePLY(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &vecPC,const std::string &dir,const std::vector<std::string> vecPaths)
{
    int fails = 0;
    std::vector<std::string>::const_iterator  itPath = vecPaths.begin();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::const_iterator  itPC = vecPC.begin();
    for (itPath, itPC; itPath != vecPaths.end(), itPC != vecPC.end(); ++itPath, ++itPC)
        if (!savePLY(*itPC, dir + "/" + *itPath))
            fails++;
    return fails;
}

int saveSTL(const std::vector<pcl::PolygonMesh::Ptr> &vecMesh,const std::string &dir,const std::vector<std::string> vecPaths)
{
    int fails = 0;
    std::vector<std::string>::const_iterator  itPath = vecPaths.begin();
    std::vector<pcl::PolygonMesh::Ptr>::const_iterator  itPC = vecMesh.begin();
    for (itPath, itPC; itPath != vecPaths.end(), itPC != vecMesh.end(); ++itPath, ++itPC)
        if (!saveSTL(*itPC, dir + "/" + *itPath))
            fails++;
    return fails;
}

int saveVTK(const std::vector<pcl::PolygonMesh::Ptr> &vecMesh,const std::string &dir,const std::vector<std::string> vecPaths)
{
    int fails = 0;
    std::vector<std::string>::const_iterator  itPath = vecPaths.begin();
    std::vector<pcl::PolygonMesh::Ptr>::const_iterator  itPC = vecMesh.begin();
    for (itPath, itPC; itPath != vecPaths.end(), itPC != vecMesh.end(); ++itPath, ++itPC)
        if (!saveVTK(*itPC, dir + "/" + *itPath))
            fails++;
    return fails;
}

}
