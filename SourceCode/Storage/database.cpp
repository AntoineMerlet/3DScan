#include "database.h"
#include "logger.h"
/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to get the vector of Raw (no processing) Point Clouds from the DataBase
/// @return The vector of Point Clouds
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>  DataBase::getRawPCs()
{
    try
    {
        return rawPCs;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return {};
    }
}


/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to get the vector of registered Point Clouds from the DataBase
/// @return The vector of Poiant Clouds
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> DataBase::getRegisteredPCs()
{
    try
    {
        return registeredPCs;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return {};
    }
}



/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to get the vector of meshed Point Clouds from the DataBase
/// @return The vector of Meshs
std::vector<pcl::PolygonMesh::Ptr> DataBase::getMeshedPCs()
{
    try
    {
        return meshedPCs;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return {};
    }
}


/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to get several raw Point Clouds from the DataBase, specified by index
/// @param li The list of index of wanted Point Clouds
/// @return The vector of wanted Point Clouds
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>  DataBase::getRawPCs(std::list<int> li)
{
    try
    {
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> resvec;
        std::list<int>::iterator it = li.begin();
        for (it; it != li.end(); it++)
            resvec.push_back(rawPCs.at(*it));
        return resvec;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return {};
    }
}

/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to get several registered Point Clouds from the DataBase, specified by index
/// @param li The list of index of wanted Point Clouds
/// @return The vector of wanted Point Clouds
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> DataBase::getRegisteredPCs(std::list<int> li)
{
    try
    {
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> resvec;
        std::list<int>::iterator it = li.begin();
        for (it; it != li.end(); it++)
            resvec.push_back(registeredPCs.at(*it));
        return resvec;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return {};
    }
}

/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to get several meshed Point Clouds from the DataBase, specified by index
/// @param li The lost of index of wanted meshs
/// @return The vector of wanted meshs
std::vector<pcl::PolygonMesh::Ptr> DataBase::getMeshedPCs(std::list<int> li)
{
    try
    {
        std::vector<pcl::PolygonMesh::Ptr> resvec;
        std::list<int>::iterator it = li.begin();
        for (it; it != li.end(); it++)
            resvec.push_back(meshedPCs.at(*it));
        return resvec;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return {};
    }
}

/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to get one raw Point Cloud from the DataBase, specified by index
/// @param idx The index of the Point Cloud to return
/// @return The wanted Point Cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  DataBase::getRawPC(const int &idx)
{
    try
    {
        return rawPCs.at(idx);
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return nullptr;
    }
}

/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to get one registered Point Cloud from the DataBase, specified by index
/// @param idx The index of the Point Cloud to return
/// @return The wanted Point Cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr DataBase::getRegisteredPC(const int &idx)
{
    try
    {
        return registeredPCs.at(idx);
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return nullptr;
    }
}

/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to get one mesh from the DataBase, specified by index
/// @param idx The index of the mesh to return
/// @return The wanted mesh
pcl::PolygonMesh::Ptr DataBase::getMeshedPC(const int &idx)
{
    try
    {
        return meshedPCs.at(idx);
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return nullptr;
    }
}


/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to add one raw Point Cloud to the DataBase
/// @param pc The Point Cloud to add
/// @return true if success, false if error
bool DataBase::addRawPC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc)
{
    try
    {
        rawPCs.push_back(pc);
        LOG("Added 1 rawPC in DB. Size: " + std::to_string(rawPCs.size()));
        return true;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return false;
    }
}


/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to add one registered Point Cloud to the DataBase
/// @param pc The Point Cloud to add
/// @return true if success, false if error
bool DataBase::addRegisteredPC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc)
{
    try
    {
        registeredPCs.push_back(pc);
        LOG("Added 1 registeredPC in DB. Size: " + std::to_string(registeredPCs.size()));
        return true;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return false;
    }
}

/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to add one mesh to the DataBase
/// @param mesh The mesh to add
/// @return true if success, false if error
bool DataBase::addMeshedPC(const pcl::PolygonMesh::Ptr &mesh)
{
    try
    {
        meshedPCs.push_back(mesh);
        LOG("Added 1 Mesh in DB. Size: " + std::to_string(meshedPCs.size()));
        return true;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return false;
    }
}

/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to change the value of a raw Point Cloud in the DataBase, given by index
/// @param pc The new Point Cloud value
/// @param idx The index of the Point Cloud to change
/// @return true if success, false if error
bool DataBase::replaceRawPC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc, const int &idx)
{
    try
    {
        rawPCs.at(idx) = pc;
        LOG("1 rawPC replaced DB. Size: " + std::to_string(registeredPCs.size()));
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return false;
    }
}

/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to change the value of a registered Point Cloud in the DataBase, given by index
/// @param pc The new Point Cloud value
/// @param idx The index of the Point Cloud to change
/// @return true if success, false if error
bool DataBase::replaceRegisteredPC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc, const int &idx)
{
    try
    {
        registeredPCs.at(idx) = pc;
        LOG("1 registeredPC replaced DB");
        return true;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return false;
    }
}

/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to change the value of a mesh in the DataBase, given by index
/// @param pc The new mesh value
/// @param idx The index of the mesh to change
/// @return true if success, false if error
bool DataBase::replaceMeshedPC(const pcl::PolygonMesh::Ptr &mesh, const int &idx)
{
    try
    {
        meshedPCs.at(idx) = mesh;
        LOG("1 mesh replaced DB");
        return true;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return false;
    }
}

/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to delete a raw Point Cloud in the DataBase, given by index
/// @param idx The index of the Point Cloud to delete
/// @return true if success, false if error
bool DataBase::removeRawPC(const int &idx)
{
    try
    {
        rawPCs.erase(rawPCs.begin()+idx);
        LOG("1 rawPC removed from DB. Size: " + std::to_string(rawPCs.size()));
        return true;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return false;
    }
}

/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to delete a registered Point Cloud in the DataBase, given by index
/// @param idx The index of the Point Cloud to delete
/// @return true if success, false if error
bool DataBase::removeRegisteredPC(const int &idx)
{
    try
    {
        registeredPCs.erase(registeredPCs.begin()+idx);
        LOG("1 registeredPC removed from DB. Size: " + std::to_string(registeredPCs.size()));
        return true;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return false;
    }
}

/// @author: Antoine Merlet
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function used to delete a mesh in the DataBase, given by index
/// @param idx The index of the mesh to delete
/// @return true if success, false if error
bool DataBase::removeMeshedPC(const int &idx)
{
    try
    {
        meshedPCs.erase(meshedPCs.begin()+idx);
        LOG("1 mesh removed from DB. Size: " + std::to_string(meshedPCs.size()));
        return true;
    }
    catch (const std::exception &e)
    {
        LOG("Error: "+ *e.what());
        return false;
    }
}
