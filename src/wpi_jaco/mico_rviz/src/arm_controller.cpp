#include "ros/ros.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <mico_rviz/arm_commandAction.h>
#include <actionlib/server/simple_action_server.h>

#include <control_msgs/GripperCommandActionGoal.h>



moveit::planning_interface::MoveGroup *group;
moveit::planning_interface::MoveGroup *hand;
actionlib::SimpleActionServer<mico_rviz::arm_commandAction> *aServer;
mico_rviz::arm_commandResult result;
ros::Publisher pub;
ros::Publisher planning_pub;
planning_scene_monitor::PlanningSceneMonitorPtr psm;



double offsetConst=.25;
bool closed =false;
const int maxAttempts =20; 

void quaternion(float roll, float pitch, float yaw, float q[4]){

  double t0 = std::cos(yaw * 0.5);
  double t1 = std::sin(yaw * 0.5);
  double t2 = std::cos(roll * 0.5);
  double t3 = std::sin(roll * 0.5);
  double t4 = std::cos(pitch * 0.5);
  double t5 = std::sin(pitch * 0.5);

  q[0] = t0 * t2 * t4 + t1 * t3 * t5;
  q[1] = t0 * t3 * t4 - t1 * t2 * t5;
  q[2] = t0 * t2 * t5 + t1 * t3 * t4;
  q[3] = t1 * t2 * t4 - t0 * t3 * t5;

} 

int move(float q[4],float x,float y, float z){
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w=q[0];
  target_pose1.orientation.x=q[1];
  target_pose1.orientation.y=q[2];
  target_pose1.orientation.z=q[3];
  target_pose1.position.x = x;
  target_pose1.position.y = y;
  target_pose1.position.z = z;

  group->setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroup::Plan my_plan;

  bool success = false;
  for(int i= 0; i < maxAttempts; ++i)
  {
    success = group->plan(my_plan);
    if(success)
    {
      ROS_INFO("Found a plan in iteration #%d", i);
      break;
    } 
  }

  if(!success) 
  {
    ROS_INFO("Did not find a plan after %d iterations.", maxAttempts);
  }else{
    ROS_INFO("Executing move");
    group->execute(my_plan);
    ROS_INFO("Finished move");    
    sleep(2);
  }

  if(success){
    return 1;
  }
  else{
    return 0;
  }
}

void setClosedHand (bool closed,bool grab, std::string object){
  control_msgs::GripperCommandActionGoal goal;
  goal.goal.command.max_effort=5;





  if(closed){
    goal.goal.command.position=.7; 
    if(grab){
      group->attachObject(object);
    }
  }else{
    goal.goal.command.position=0;
  }

  pub.publish(goal);
  sleep(2);

  planning_scene_monitor::LockedPlanningSceneRW planning_scene = planning_scene_monitor::LockedPlanningSceneRW(psm);
  collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();

  if(closed){
    moveit_msgs::PlanningScene newSceneDiff;
    moveit_msgs::AllowedCollisionMatrix matrixMsg;
    collision_detection::CollisionRequest collision_request;
    //collision_request.contacts=true;
    collision_detection::CollisionResult collision_result;
    planning_scene->checkSelfCollision(collision_request, collision_result);

    collision_detection::CollisionResult::ContactMap::const_iterator it2;
    ROS_INFO("Checking collisions");

    // for(it2 = collision_result.contacts.begin();
    //     it2 != collision_result.contacts.end();
    //     ++it2)
    // {
    //   acm.setEntry(it2->first.first, it2->first.second, true);
    //   ROS_INFO("COLLISION");
    // }

    acm.setEntry(object, "mico_link_finger_1", true);
    acm.setEntry(object, "mico_link_finger_2", true);
    acm.setEntry(object, "mico_link_finger_tip_1", true);
    acm.setEntry(object, "mico_link_finger_tip_2", true);

    collision_result.clear();

    acm.getMessage(matrixMsg);
    newSceneDiff.is_diff = true;
    newSceneDiff.allowed_collision_matrix =matrixMsg; 

    planning_pub.publish(newSceneDiff);

  }


  if(!closed && grab){
    group->detachObject(object);
    sleep(2);
  }
}


void inputCallback(const mico_rviz::arm_commandGoalConstPtr & goal)
{
  float x=goal->x;
  float y=goal->y;
  float z=goal->z;
  bool grab=goal->grab;
  std::string name=goal->name;

  if(!closed){
    closed = !closed;
    setClosedHand(closed,false,name);
  }

  ROS_INFO("I heard: [%f %f %f]", x,y,z);

  geometry_msgs::Pose target_pose1;
 

  float dist= sqrt(x*x+y*y);

  float delta=(dist-offsetConst)/dist;

  float dx=(x/dist)*offsetConst;
  float dx2=(x/dist)*(offsetConst-0.1);
  float dy=(y/dist)*offsetConst;
  float dy2=(y/dist)*(offsetConst-0.1);

  float angle;
  if(x==0 && y==0){
    angle=M_PI/2;
  }else{
    if(x==0){
      angle=M_PI/2*(fabs(y)/y);
    }else{
      angle= std::atan(y/x);
    }
  }
  if(x>=0){
    angle += M_PI;
    if(angle>M_PI){
      angle -= 2*M_PI;
    }
  }

  float q[4];

  ROS_INFO("Angle: [%f]", angle);

  quaternion(M_PI/2,0,angle+M_PI/2,q);

  bool res =true;
  if(grab){
    res = res && move(q,x-dx,y-dy,z);    
    setClosedHand(false,false,name);
    closed=false;
  }

  res= res &&  move(q,x-dx2,y-dy2,z);

  if(res){
    closed = !closed;
    setClosedHand(closed,true,name);
  }

  result.result=res ? 1 : 0;

  aServer->setSucceeded(result);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "arm_controller");

  ros::NodeHandle n;

  group = new moveit::planning_interface::MoveGroup("arm");
  hand = new moveit::planning_interface::MoveGroup("eef");
  pub = n.advertise<control_msgs::GripperCommandActionGoal>("/mico_hand/fingers_controller_radian/gripper/goal",1000);
  planning_pub = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1); 

  group->allowLooking(true);
  group->allowReplanning(true);
  group->setNumPlanningAttempts(3);

  psm = planning_scene_monitor::PlanningSceneMonitorPtr(
        new planning_scene_monitor::PlanningSceneMonitor("robot_description")); 

  aServer=new actionlib::SimpleActionServer<mico_rviz::arm_commandAction>(n,"arm_controller",boost::bind(&inputCallback,_1),false);

  aServer->start();
  

  ros::spin();

  return 0;
}