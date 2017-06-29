#include <iostream>
#include "ros/ros.h"
#include "colorblob/colorblob.h"


using namespace std;

int main(int argc, char **argv)
{
       ros::init(argc, argv, "Colorblob");
       
       Colorblob colorblob;

       colorblob.run();
}
