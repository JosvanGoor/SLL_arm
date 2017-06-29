#include <pioneercontroller2/motioncontroller.h>
#include <iostream>

using namespace std;

void MotionController::setOptions(int option1, int option2)
{
    d_lock.lock();
    if (not connected())
    {
        ROS_ERROR("No connection to robot. Not setting options");
        d_lock.unlock();
        return;
    }

    d_robot->lock();
    switch (option1)
    {
    	case 0:
            ROS_INFO("Set Rotation Velocity Max Speed to %d", option2);
    		d_robot->setRotVelMax(option2);
			break;
    	case 1:
            ROS_INFO("Set Rotation Acceleration to %d", option2);
			d_robot->setRotAccel(option2);
    		break;
    	case 2:
            ROS_INFO("Set Rotation Deceleration to %d", option2);
			d_robot->setRotDecel(option2);
			break;
    	case 3:
            ROS_INFO("Set Translational Acceleration to %d", option2);
    		d_robot->setTransAccel(option2);
    		break;
    	case 4:
            ROS_INFO("Set Translational Deceleration to %d", option2);
    		d_robot->setTransDecel(option2);
    		break;
    }
    d_robot->unlock();
    d_lock.unlock();
}
