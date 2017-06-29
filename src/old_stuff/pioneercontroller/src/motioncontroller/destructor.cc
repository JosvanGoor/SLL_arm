#include <pioneercontroller2/motioncontroller.h>

MotionController::~MotionController()
{
    disconnectRobot();
    stop();
    s_instance = 0;
}
