#ifndef KINECT_V2_H
#define KINECT_V2_H

#include "IO/sensors.h"
#include "IO/kinect2_grabber.h"

namespace IO {

class Kinect_V2: public Sensors
{
public:
    Kinect_V2();
    bool isAvailable();
    bool setupSensor();
    bool startSensor();
    bool stopSensor();

    boost::function<void( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& )> pointCloudCallback
    = [this]( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& ptr )
    {
        boost::mutex::scoped_lock lock(mutex);
        //mf_SetMvPointCloud(ptr);
    };

protected:
    boost::mutex                            mutex;
    boost::shared_ptr<pcl::Kinect2Grabber>  grabber;
    boost::signals2::connection             connection;
};

}

#endif // KINECT_V2_H
