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
};

}

#endif // KINECT_V2_H
