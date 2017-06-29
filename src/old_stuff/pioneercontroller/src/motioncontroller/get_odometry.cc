#include <pioneercontroller2/motioncontroller.h>
#include <iostream>

using namespace std;

/** Python time.time() compatible time retrieval. The current time is obtained
  * using gettimeofday and then converted to a floating point number as the
  * number of seconds since the epoch.
  *
  * @return The number of seconds sinds 1-1-1980 00:00:00
  */
double _pythonTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);

    return (tv.tv_sec) + (static_cast<long double>(tv.tv_usec) / 1000000.0);
}

/** Obtain odometry information from the Pioneer. Two formats are available: a
  * string or an Odometry struct. The latter gives more specific and structured
  * information, the former is maintaned for backwards compatibility with the
  * BORG pioneer controller.
  * The string can be omitted by passing a 0-pointer.
  * 
  * @param msg Pointer to a character array for the odometry string
  * @param odo Odometry struct that will be filled with the Odometry info The
  *            contents of this struct will be unchanged when no connection is
  *            available.
  */
void MotionController::getOdometry(char *msg, Odometry &odo)
{
	d_lock.lock();
    if (connected())
    {
        // First, read data from robot
        d_robot->lock();
        odo.x = d_robot->getX();
        odo.y = d_robot->getY();
        odo.theta = d_robot->getTh();
        odo.voltage = d_robot->getBatteryVoltage();
        odo.time = _pythonTime();
        odo.rot_vel = d_robot->getRotVel();
        odo.trans_vel = d_robot->getVel();
        d_robot->unlock();
        
        // Transform using the stored transform: X, Y and theta
        float x = odo.x;
        float y = odo.y;
        odo.x = (x * cos(d_tf_theta) - y * sin(d_tf_theta)) + d_tf_x;
        odo.y = (x * sin(d_tf_theta) + y * cos(d_tf_theta)) + d_tf_y;
        odo.theta += d_tf_theta / TO_RAD;
        
        // Store last odometry to update transform when disconnected
        d_last_odometry = odo;

        // Generate odometry string
        if (msg != 0)
            sprintf(msg, "%f %f %f %f %f\n", odo.x, odo.y, odo.theta, odo.voltage, odo.time);
    }
    d_lock.unlock();
}
