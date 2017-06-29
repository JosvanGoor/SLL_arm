#include <pioneercontroller2/motioncontroller.h>
#include <iostream>
#include <sstream>
#include <cstring>

using namespace std;

void MotionController::getStatusString(char * msg)
{
    ostringstream out;
    d_lock.lock();
    if (connected())
    {
        d_robot->lock();
        out << "Connected: yes" << endl;
        out << "X: " << d_robot->getX() << endl
            << "Y: " << d_robot->getY() << endl
            << "Theta: " << d_robot->getTh() << endl
            << "Battery: " << d_robot->getBatteryVoltage() << endl;
        d_robot->unlock();
    }
    else
    {
        out << "Connected: no\n";
    }
    if (d_error != NO_ERROR && d_error != INITIALIZING)
        out << "Emergency: yes\nEMERGENCY\n";
    else
        out << "Emergency: no\n";
        
    d_lock.unlock();

    out << "Version: 2.0" << endl;
    char const *buf = out.str().c_str();
    strcpy(msg, buf);
}
