#include "kinect_v2.h"

namespace IO {

Kinect_V2::Kinect_V2()
{
    setSensorID("KINECTV2");
    setSensorName("Kinect_V2");

    setupSensor();
}

bool Kinect_V2::isAvailable()
{
    return grabber->isAvailable();
}

bool Kinect_V2::setupSensor()
{
    grabber = boost::make_shared<pcl::Kinect2Grabber>();
    connection = grabber->registerCallback(pointCloudCallback);
    return true;
}

bool Kinect_V2::startSensor()
{
    if(isAvailable())
    {
        grabber->start();
        return true;
    }
    return false;
}

bool Kinect_V2::stopSensor()
{
    if(isAvailable())
    {
        grabber->stop();
        return true;
    }
    return false;
}
}
