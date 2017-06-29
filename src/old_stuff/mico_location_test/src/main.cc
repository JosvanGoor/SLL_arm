#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"

#include <mico_location_test/objectLocation.h>

#include <sstream>

using namespace std;
using namespace ros;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mico_testing");
    ros::NodeHandle nh("~");
    
    Publisher publisher = nh.advertise<mico_location_test::objectLocation>("objectDetection/location", 1);

    mico_location_test::objectLocation msg;
    msg.x -= 0.0f;
    msg.y = -0.50f;
    msg.z = 0.10f;
    msg.thetaX = 1.55784;
    msg.thetaY = 0.0;
    msg.thetaZ = 0.0;
   
    ros::Rate loop_rate(10);
    while (ros::ok())
    { 
        cin.ignore();
        publisher.publish(msg); 
        ros::spinOnce();
        loop_rate.sleep();
    }
}

