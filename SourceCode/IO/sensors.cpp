#include "sensors.h"

namespace IO {

Sensors::Sensors()
{

}

Sensors::~Sensors()
{

}

void Sensors::setCurrentPC(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr PC)
{
    if(PC != nullptr && PC->points.size() > 0)
        currentPC = PC->makeShared();
}

}

