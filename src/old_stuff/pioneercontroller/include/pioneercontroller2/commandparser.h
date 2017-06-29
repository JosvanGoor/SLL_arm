#ifndef INCL_COMMANDPARSER_H
#define INCL_COMMANDPARSER_H

#include <string>
#include <vector>
#include "types.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>


class CommandParser {
    public:
        CommandParser();
        ~CommandParser();
        pairActionpairInt parseCommand(std::string const &msg);
        pairActionpairDouble parseTwistCommand(geometry_msgs::Twist::ConstPtr const &msg);
};


#endif
