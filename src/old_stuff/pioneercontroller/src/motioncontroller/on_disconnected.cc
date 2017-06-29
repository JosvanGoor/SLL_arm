#include <pioneercontroller2/motioncontroller.h>

// lost connection, so just exit
void MotionController::on_disconnected()
{
    ROS_ERROR("Robot connection handler: Lost connection");

    d_lock.lock();
    d_error = CONNECTION_LOST;
    d_lock.unlock();
}
