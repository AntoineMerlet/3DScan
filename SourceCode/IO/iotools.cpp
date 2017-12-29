#include "iotools.h"
#include <exception>
#include "logger.h"

namespace IO {

/// @author: Antoine Merlet
/// @date: 15-12-2017
/// @version 1.1
///
/// @brief Function used to load the .PCD file given by path and returning the loaded Point Cloud.
/// @param path The path of the .PCD file to load.
/// @return The loaded point cloud in XYZ coordinates and RGB colors, as pointer. Or nullptr if errors while loading.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPCD(const std::string &path)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrPC (new pcl::PointCloud<pcl::PointXYZRGB>());
    try
    {
        pcl::io::loadPCDFile(path, *ptrPC);

        LOG("Loaded " + path);
        return ptrPC;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return nullptr;
    }
}
/// @author: Antoine Merlet
/// @date: 15-12-2017
/// @version 1.1
///
/// @brief Function used to load the .PLY file given by path and returning the loaded Point Cloud.
/// @param path The path of the .PLY file to load.
/// @return The loaded point cloud in XYZ coordinates and RGB colors, as pointer. Or nullptr if errors while loading.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPLY(const std::string &path)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrPC (new pcl::PointCloud<pcl::PointXYZRGB>());
    try
    {
        pcl::io::loadPLYFile(path, *ptrPC);
        LOG("Loaded " + path);
        return ptrPC;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return nullptr;
    }
}

/// @author: Antoine Merlet
/// @date: 15-12-2017
/// @version 1.1
///
/// @brief Function used to load the .STL file given by path and returning the loaded mesh.
/// @param path The path of the file to load.
/// @return The loaded Mesg
///  pointer. Or nullptr if errors while loading.
pcl::PolygonMesh::Ptr loadSTL(const std::string &path)
{
    pcl::PolygonMesh::Ptr ptrMesh (new pcl::PolygonMesh());
    try
    {
        pcl::io::loadPolygonFileSTL(path, *ptrMesh);
        LOG("Loaded " + path);
        return ptrMesh;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return nullptr;
    }
}

/// @author: Antoine Merlet
/// @date: 15-12-2017
/// @version 1.1
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
        LOG("Loaded " + path);
        return ptrMesh;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return nullptr;
    }
}

/// @author: Antoine Merlet
/// @date: 26-12-2017
/// @version 1.1
///
/// @brief Function used to load all the .PCD file given the vector of path.
/// @param path The vector of paths of the files to load.
/// @return The loaded PointCloud pointer.
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

/// @author: Antoine Merlet
/// @date: 26-12-2017
/// @version 1.1
///
/// @brief Function used to load all the .PLY file given the vector of path.
/// @param path The vector of paths of the files to load.
/// @return The loaded PointCloud pointer.
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

/// @author: Antoine Merlet
/// @date: 26-12-2017
/// @version 1.1
///
/// @brief Function used to load all the .STL file given the vector of path.
/// @param path The vector of paths of the files to load.
/// @return The loaded mesh pointer.
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

/// @author: Antoine Merlet
/// @date: 26-12-2017
/// @version 1.1
///
/// @brief Function used to load all the .VTK file given the vector of path.
/// @param path The vector of paths of the files to load.
/// @return The loaded mesh pointer.
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


/// @author: Antoine Merlet
/// @date: 26-12-2017
/// @version 1.0
///
/// @brief Function used to save  the Point Cloud to the specified path as       PCD file.
/// @param ptrPC The Point Cloud to save
/// @param path The path were to save the Point Cloud
/// @return True if saved, false if error
bool savePCD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ptrPC,const std::string &path)
{
    try
    {
        pcl::io::savePCDFile(path, *ptrPC);
        LOG("Saved at" + path);
        return true;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return false;
    }
}

/// @author: Antoine Merlet
/// @date: 26-12-2017
/// @version 1.0
///
/// @brief Function used to save  the Point Cloud to the specified path as PLY file.
/// @param ptrPC The Point Cloud to save
/// @param path The path were to save the Point Cloud
/// @return True if saved, false if error
bool savePLY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ptrPC,const std::string &path)
{
    try
    {
        pcl::io::savePLYFile(path, *ptrPC);
        LOG("Saved at" + path);
        return true;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return false;
    }
}

/// @author: Antoine Merlet
/// @date: 26-12-2017
/// @version 1.0
///
/// @brief Function used to save  the Mesh to the specified path as STL file.
/// @param ptrMesh The Mesh to save
/// @param path The path were to save the Mesh
/// @return True if saved, false if error
bool saveSTL(pcl::PolygonMesh::Ptr &ptrMesh,const std::string &path)
{
    try
    {
        pcl::io::savePolygonFileSTL(path, *ptrMesh);
        LOG("Saved at" + path);
        return true;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return false;
    }
}

/// @author: Antoine Merlet
/// @date: 26-12-2017
/// @version 1.0
///
/// @brief Function used to save  the Mesh to the specified path as VTK file.
/// @param ptrMesh The Mesh to save
/// @param path The path were to save the Mesh
/// @return True if saved, false if error
bool saveVTK(pcl::PolygonMesh::Ptr &ptrMesh,const std::string &path)
{
    try
    {
        pcl::io::savePolygonFileVTK(path, *ptrMesh);
        LOG("Saved at" + path);
        return true;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return false;
    }
}

/// @author: Antoine Merlet
/// @date: 26-12-2017
/// @version 1.0
///
/// @brief Function used to save the Point Clouds to the specified paths as PCD file.
/// @param vecPC The vector of Point Cloud  to save
/// @param vecPaths The vector of paths were to save the multiple Point Clouds
/// @return The number of errors while saving
int savePCD(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &vecPC,const std::string &dir, std::vector<std::string> vecPaths)
{
    int fails = 0;
    std::vector<std::string>::iterator  itPath = vecPaths.begin();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator  itPC = vecPC.begin();
    for (itPath, itPC; itPath != vecPaths.end(), itPC != vecPC.end(); ++itPath, ++itPC)
        if (!savePCD(*itPC, dir + "/" + *itPath))
            fails++;
    return fails;
}

/// @author: Antoine Merlet
/// @date: 26-12-2017
/// @version 1.0
///
/// @brief Function used to save the Point Clouds to the specified paths as PLY file.
/// @param vecPC The vector of Point Cloud  to save
/// @param vecPaths The vector of paths were to save the multiple Point Clouds
/// @return The number of errors while saving
int savePLY(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &vecPC,std::string &dir,std::vector<std::string> vecPaths)
{
    int fails = 0;
    std::vector<std::string>::iterator  itPath = vecPaths.begin();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator  itPC = vecPC.begin();
    for (itPath, itPC; itPath != vecPaths.end(), itPC != vecPC.end(); ++itPath, ++itPC)
        if (!savePLY(*itPC, dir + "/" + *itPath))
            fails++;
    return fails;
}


/// @author: Antoine Merlet
/// @date: 26-12-2017
/// @version 1.0
///
/// @brief Function used to save the Meshs to the specified paths as STL file.
/// @param vecMesh The vector of Meshs  to save
/// @param vecPaths The vector of paths were to save the multiple Meshs
/// @return The number of errors while saving
int saveSTL(std::vector<pcl::PolygonMesh::Ptr> &vecMesh,const std::string &dir, std::vector<std::string> vecPaths)
{
    int fails = 0;
    std::vector<std::string>::iterator  itPath = vecPaths.begin();
    std::vector<pcl::PolygonMesh::Ptr>::iterator  itPC = vecMesh.begin();
    for (itPath, itPC; itPath != vecPaths.end(), itPC != vecMesh.end(); ++itPath, ++itPC)
        if (!saveSTL(*itPC, dir + "/" + *itPath))
            fails++;
    return fails;
}

/// @author: Antoine Merlet
/// @date: 26-12-2017
/// @version 1.0
///
/// @brief Function used to save the Meshs to the specified paths as VTK file.
/// @param vecMesh The vector of Meshs  to save
/// @param vecPaths The vector of paths were to save the multiple Meshs
/// @return The number of errors while saving
int saveVTK(std::vector<pcl::PolygonMesh::Ptr> &vecMesh,const std::string &dir, std::vector<std::string> vecPaths)
{
    int fails = 0;
    std::vector<std::string>::iterator  itPath = vecPaths.begin();
    std::vector<pcl::PolygonMesh::Ptr>::iterator  itPC = vecMesh.begin();
    for (itPath, itPC; itPath != vecPaths.end(), itPC != vecMesh.end(); ++itPath, ++itPC)
        if (!saveVTK(*itPC, dir + "/" + *itPath))
            fails++;
    return fails;
}

}
