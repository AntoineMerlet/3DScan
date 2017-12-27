#include "database.h"

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>  DataBase::getRawPCs()
{
    try
    {
        return rawPCs;
    }
    catch (const std::exception &e)
    {

        return {};
    }
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> DataBase::getRegisteredPCs()
{
    try
    {
        return registeredPCs;
    }
    catch (const std::exception &e)
    {

        return {};
    }
}

std::vector<pcl::PolygonMesh::Ptr> DataBase::getMeshedPCs()
{
    try
    {
        return meshedPCs;
    }
    catch (const std::exception &e)
    {

        return {};
    }
}

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

        return {};
    }
}

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

        return {};
    }
}

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

        return {};
    }
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr  DataBase::getRawPC(const int &idx)
{
    try
    {
        return rawPCs.at(idx);
    }
    catch (const std::exception &e)
    {

        return nullptr;
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr DataBase::getRegisteredPC(const int &idx)
{
    try
    {
        return registeredPCs.at(idx);
    }
    catch (const std::exception &e)
    {

        return nullptr;
    }
}

pcl::PolygonMesh::Ptr DataBase::getMeshedPC(const int &idx)
{
    try
    {
        return meshedPCs.at(idx);
    }
    catch (const std::exception &e)
    {

        return nullptr;
    }
}

bool DataBase::addRawPC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc)
{
    try
    {
        rawPCs.push_back(pc);
        return true;
    }
    catch (const std::exception &e)
    {

        return false;
    }
}

bool DataBase::addRegisteredPC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc)
{
    try
    {
        registeredPCs.push_back(pc);
        return true;
    }
    catch (const std::exception &e)
    {

        return false;
    }
}
bool DataBase::addMeshedPC(const pcl::PolygonMesh::Ptr &mesh)
{
    try
    {
        meshedPCs.push_back(mesh);
        return true;
    }
    catch (const std::exception &e)
    {

        return false;
    }
}
// Replace the PC at given location with PC given as parameter
bool DataBase::replaceRawPC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc, const int &idx)
{
    try
    {
        rawPCs.at(idx) = pc;
        return true;
    }
    catch (const std::exception &e)
    {

        return false;
    }
}

bool DataBase::replaceRegisteredPC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc, const int &idx)
{
    try
    {
        registeredPCs.at(idx) = pc;
        return true;
    }
    catch (const std::exception &e)
    {

        return false;
    }
}

bool DataBase::replaceMeshedPC(const pcl::PolygonMesh::Ptr &mesh, const int &idx)
{
    try
    {
        meshedPCs.at(idx) = mesh;
        return true;
    }
    catch (const std::exception &e)
    {

        return false;
    }
}

// Delte the PC at given location
bool DataBase::removeRawPC(const int &idx)
{
    try
    {
       rawPCs.erase(rawPCs.begin()+idx);
       return true;
    }
    catch (const std::exception &e)
    {

        return false;
    }
}

bool DataBase::removeRegisteredPC(const int &idx)
{
    try
    {
       registeredPCs.erase(registeredPCs.begin()+idx);
       return true;
    }
    catch (const std::exception &e)
    {

        return false;
    }
}

bool DataBase::removeMeshedPC(const int &idx)
{
    try
    {
       meshedPCs.erase(meshedPCs.begin()+idx);
       return true;
    }
    catch (const std::exception &e)
    {

        return false;
    }
}
