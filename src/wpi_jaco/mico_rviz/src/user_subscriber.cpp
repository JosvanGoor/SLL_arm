#include "ros/ros.h"
#include "mico_rviz/user_input.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <mico_rviz/arm_commandAction.h>
#include <actionlib/client/simple_action_client.h>

#include <cstdlib>

moveit::planning_interface::MoveGroup *group;
moveit::planning_interface::MoveGroup *hand;
ros::Publisher pub;  
double offsetConst=.2;
bool closed =false;
const int maxAttempts =10; 

actionlib::SimpleActionClient<mico_rviz::arm_commandAction> *aClient;


void inputCallback(const mico_rviz::user_input::ConstPtr& msg)
{


  ROS_INFO("Start Callback");  
  float x=msg->x;
  float y=msg->y;
  float z=msg->z;
  float radius=msg->radius;
  float length=msg->length;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group->getPlanningFrame();

  /* The id of the object is used to identify it. */

  std::stringstream obj_name;
  obj_name << rand();
  collision_object.id = std::string(obj_name.str());

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = length;
  primitive.dimensions[1] = radius -.01;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose cylinder_pose;
  cylinder_pose.orientation.w = 1.0;
  cylinder_pose.position.x =  x;
  cylinder_pose.position.y = y;
  cylinder_pose.position.z =  z;


  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(cylinder_pose);
  collision_object.operation = collision_object.ADD;

  pub.publish(collision_object);
  
  sleep(1.0);

  mico_rviz::arm_commandGoal goal;

  goal.x=x;
  goal.y=y;
  goal.z=z;
  goal.grab=true;
  goal.name= collision_object.id; 

  aClient->sendGoal(goal);

  goal.x=0.443;
  goal.y=0.354;
  goal.z +=0.02;
  goal.grab=false;

  ROS_INFO("Calling server"); 

  bool success = aClient->waitForResult(ros::Duration(90.0));
  sleep(3);

  ROS_INFO("---- DONE WITH SERVER ----"); 
  ROS_INFO("%d",aClient->getResult()->result);  

  if(success && aClient->getResult()->result == 1){
    aClient->sendGoal(goal);
    aClient->waitForResult(ros::Duration(30.0));
  }

  // std::vector<std::string> object_ids;
  // object_ids.push_back(collision_object.id);
  // planning_scene_interface.removeCollisionObjects(object_ids);

  // sleep(4.0);

  collision_object.operation = collision_object.REMOVE;
  pub.publish(collision_object);

  ROS_INFO("Finished"); 
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "user_input");
  ros::AsyncSpinner spinner(0);
  spinner.start();



  ros::NodeHandle n;

  group = new moveit::planning_interface::MoveGroup("arm");

  ros::Subscriber sub = n.subscribe("user_input", 1000, inputCallback);
  pub =n.advertise<moveit_msgs::CollisionObject>("collision_object",1000);  

  aClient=new actionlib::SimpleActionClient<mico_rviz::arm_commandAction>("arm_controller",true);
  aClient->waitForServer();

  ROS_INFO("---- USER SUBSCRIBER STARTED -----");  


  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group->getPlanningFrame();

  /* The id of the object is used to identify it. */
  collision_object.id = "table";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 2.0;
  primitive.dimensions[1] = 2.0;
  primitive.dimensions[2] = 0.01;


  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose cylinder_pose;
  cylinder_pose.orientation.w = 1.0;
  cylinder_pose.position.x =  0.0;
  cylinder_pose.position.y = 0;
  cylinder_pose.position.z =  -0.12;


  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(cylinder_pose);
  collision_object.operation = collision_object.ADD;

  pub.publish(collision_object);


  ros::spin();

  return 0;
}