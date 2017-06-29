#include <pioneercontroller2/motioncontroller.h>

// set error status if the connection failed
void MotionController::on_connFail(void)
{
    ROS_ERROR("Robot connection handler: failed to connect to robot");

    d_lock.lock();
    d_error = CONNECTION_FAILED;
    d_lock.unlock();
}
