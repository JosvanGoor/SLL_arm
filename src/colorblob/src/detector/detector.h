#ifndef INCLUDED_DETECTOR_H_
#define INCLUDED_DETECTOR_H_

#include <ros/ros.h>
#include <string>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>

class Detector
{
  std::string const d_botcam_topic;
  std::string const d_topcam_topic;
  cv_bridge::CvImagePtr d_cv_ptr;
  
public:
  Detector();
  void run();
  void detect();
  //static void onMouse() //TODO interaction stuff
  
private:
  ros::NodeHandle d_nh;
  ros::Subscriber d_image_sub;
  ros::ServiceClient d_mem_client;
  
};

#endif