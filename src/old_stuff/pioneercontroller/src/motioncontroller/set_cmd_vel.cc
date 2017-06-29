#include <pioneercontroller2/motioncontroller.h>
#include <iostream>

using namespace std;

void MotionController::setCmdVel(double linear, double angular)
{
    d_lock.lock();
    if (not connected())
    {
        ROS_ERROR("Not connected to robot, so not sending command");
        d_lock.unlock();
        return;
    }

    d_robot->lock();
    d_robot->setVel(linear);
    d_robot->setRotVel(angular);
    d_robot->unlock();
    d_lock.unlock();
}
