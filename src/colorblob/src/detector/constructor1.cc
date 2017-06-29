#include "detector.ih"

Detector::Detector()
:
  d_botcam_topic("/NAO/image_bottom"),
  d_topcam_topic("/NAO/image_top")
{
  ros::NodeHandle param_node("~");
  string image_topic;
  param_node.param<string>("image_topic", image_topic, \   
    d_botcam_topic.c_str());
  
  
  
  d_image_sub = d_nh.subscribe(image_topic, 1, &Detector::newImageCB, this)
}