#include "detector.ih"

void Detector::newImageCB(const sensor_msgs::ImageConstPtr& msg)
{
  d_cv_ptr = cv_bridge::toCvCopy(msg);
  
  //TODO do stuff with image, e.g. preproc
  
  detect(); //detect stuff from current image ptr
  cout << "got image\n";
}