#ifndef SENSORS_H
#define SENSORS_H

#include <map>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace IO {

class Sensors
{

public:
    Sensors();
    ~Sensors();

    virtual bool isAvailable() = 0;
    virtual bool setupSensor() = 0;
    virtual bool startSensor() = 0;
    virtual bool stopSensor() = 0;

    // Getters
    std::string getSensorID() {return sensorID;}
    std::string getSensorName() {return sensorName;}
    std::map<std::string, std::string> getSensorDetails() {return sensorDetails;}
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr getCurrentPC() {return currentPC;}

    // Setters
    void setSensorID(std::string id) {sensorID = id;}
    void setSensorDetails(std::map<std::string, std::string> mp) {sensorDetails = mp;}
    void setSensorName(std::string name) {sensorName = name;}
    void setCurrentPC(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr);

protected:

    std::string sensorID;
    std::string sensorName;
    std::map<std::string, std::string> sensorDetails;
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr currentPC;
};

}


#endif // SENSORS_H
