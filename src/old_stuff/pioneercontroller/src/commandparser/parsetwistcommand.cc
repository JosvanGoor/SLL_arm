#include <pioneercontroller2/commandparser.h>

#include <iostream>
#include <sstream>
#include <cmath>

using namespace std;

pairActionpairDouble CommandParser::parseTwistCommand(geometry_msgs::Twist::ConstPtr const &msg)
{
    double linear = msg->linear.x * 1000;    // msg: m/s -> Pioneer: mm/s
    double angular = msg->angular.z * 180 / M_PI;         // msg: radians/sec

    cout << "Linear: " << linear << ", angular: " << angular << endl;

    
    return pairActionpairDouble(TWIST, pairDouble(linear, angular));

/*
    // Angular velocity is in radions per second. 
    // The rotational speed in mm/s must be such that in
    // a rotation of 2 * Pi (= the perimeter of the circle of the wheel
    // base) is completed in this time.
    // The diameter of the circle of the wheel base is ~330mm, so the
    // perimeter of the circle is pi * 330 = 1036.73. However, since we're
    // making the rotation using two wheels at the same time, each takes
    // halve of the rotation, and should thus be halved
    const double rot_speed = 1036.73 / 2.0;
    //const double rot_speed = 1536.73 / 2.0;

    // Now adjust the linear velocity with the angular velocity to incorporate
    // the rotation.
    int left = linear + static_cast<int>(round((angular / -M_PI) * rot_speed));
    int right = linear + static_cast<int>(round((angular / M_PI) * rot_speed));

    if (abs(left) > MAX_SPEED || abs(right) > MAX_SPEED)
    {
        int max = abs(left) > abs(right) ? abs(left) : abs(right);
        float factor = static_cast<float>(max) / MAX_SPEED;

        left = static_cast<int>(round(left / factor));
        right = static_cast<int>(round(right / factor));
    }

    return pairActionpairInt(TWIST, pairInt(left, right));
*/
}
