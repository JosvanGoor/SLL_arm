#include <pioneercontroller2/motioncontroller.h>

#include <ctime>
#include <unistd.h>
#include <iostream>

using namespace std;

void MotionController::handleCommand(pairActionpairInt command)
{
    switch(command.first)
    {
        case LEFTRIGHT:
            setSpeeds(command.second.first, command.second.second); //set the speeds to the (left, right) values
            clock_gettime(CLOCK_REALTIME, &d_received_time);
            d_do_timeout = true;
            break;
        case FORWARD:
            driveForward(command.second.first);
            clock_gettime(CLOCK_REALTIME, &d_received_time);
            d_do_timeout = false;
            break;
        case TURN:
            driveTurn(command.second.first);
            clock_gettime(CLOCK_REALTIME, &d_received_time);
            d_do_timeout = false;
            break;
        case OPTIONS:
        	setOptions(command.second.first, command.second.second);
        	clock_gettime(CLOCK_REALTIME, &d_received_time);
			d_do_timeout = false;
        case NONE: //no (correct) action received //falling through
        default:   //or any other value
          timespec current_time;
          clock_gettime(CLOCK_REALTIME, &current_time);
          float elapsed_time = tdif(d_received_time, current_time);

          if ((elapsed_time > d_max_idle_time) && d_do_timeout)
              setSpeeds(0, 0); // stop when we haven't received a command for too long
    }
}
