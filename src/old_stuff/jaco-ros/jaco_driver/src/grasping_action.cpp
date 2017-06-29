#include "jaco_driver/grasping_action.h"
#include <kinova/KinovaTypes.h>
#include "jaco_driver/jaco_types.h"


namespace jaco
{
	GraspingActionServer::GraspingActionServer(JacoComm &arm_comm, ros::NodeHandle &n)
		:
		arm(arm_comm), 
    	as(n, "grasping", boost::bind(&GraspingActionServer::execute, this, _1), false),
		asPath(n, "grasping_path", boost::bind(&GraspingActionServer::executePath, this, _1), false)
    {
    	as.start();
    	asPath.start();

    	d_status = Init; // initial status when arm is started up
    	ROS_INFO_STREAM("Started grasping Action Servers");
    }

	void GraspingActionServer::moveToLeft(float z)
	{
		vector<arm_planner::Position> vPath;
		arm_planner::Position position;
		position.x = 0.1f;
		position.y = 0.2f;
		position.z = 0.12f;
		position.thetaX = 90.0f;
		position.thetaY = 90.0f;
		position.thetaZ = 0.0f;
		position.approach = false;
		position.openFingers = false;

		vPath.push_back(position);

		position.x = 0.37f;
		position.y = 0.15f;
		position.z = 0.40f;
		position.thetaX = 90.0f;
		position.thetaY = 90.0f;
		position.thetaZ = 0.0f;

		vPath.push_back(position);

		position.x = 0.35f;
		position.y = -0.05f;
		position.z = 0.40f;
		position.thetaX = 90.0f;
		position.thetaY = 20.0f;
		position.thetaZ = 0.0f;

		vPath.push_back(position);
/*
		position.x = 0.2f;
		position.y = -0.1f;
		position.z = 0.25f;
		position.thetaX = 90.0f;
		position.thetaY = 0.0f;
		position.thetaZ = 0.0f;

		vPath.push_back(position);
*/

		position.x = 0.2f;
		position.y = -0.1f;
		position.z = z;
		position.thetaX = 90.0f;
		position.thetaY = 0.0f;
		position.thetaZ = 0.0f;

		if (z < 0.1f)
			position.y = -0.4f;

		vPath.push_back(position);

		movePath(vPath);
	}

	void GraspingActionServer::moveToRight(float z)
	{
		vector<arm_planner::Position> vPath;
		arm_planner::Position position;
		position.x = -0.1f;
		position.y = 0.20f;
		position.z = 0.12f;
		position.thetaX = 90.0f;
		position.thetaY = 270.0f;
		position.thetaZ = 0.0f;
		position.approach = false;
		position.openFingers = false;

		vPath.push_back(position);

		position.x = -0.37f;
		position.y = 0.15f;
		position.z = 0.40f;
		position.thetaX = 90.0f;
		position.thetaY = -90.0f;
		position.thetaZ = 0.0f;

		vPath.push_back(position);

		position.x = -0.35f;
		position.y = -0.05f;
		position.z = 0.40f;
		position.thetaX = 90.0f;
		position.thetaY = -20.0f;
		position.thetaZ = 0.0f;

		vPath.push_back(position);
/*
		position.x = -0.2f;
		position.y = -0.1f;
		position.z = 0.25f;
		position.thetaX = 90.0f;
		position.thetaY = 0.0f;
		position.thetaZ = 0.0f;

		vPath.push_back(position);
*/
		position.x = -0.2f;
		position.y = -0.1f;
		position.z = z;
		position.thetaX = 90.0f;
		position.thetaY = 0.0f;
		position.thetaZ = 0.0f;

		if (z < 0.1f)
			position.y = -0.4f;

		vPath.push_back(position);

		movePath(vPath);
	}

    float toRad(float deg)
    {
    	return deg * (M_PI / 180.0f);
    }

    bool checkPosition(JacoPose wayPoint, JacoPose position)
    {
    	float error = 0.02f; // allow error of x cm

    	if (fabs(wayPoint.X - position.X) <= error and
    			fabs(wayPoint.Y - position.Y) <= error and
				fabs(wayPoint.Z - position.Z) <= error)
    		return true;
    	else
    		return false;
    }

    bool checkRotation(JacoPose wayPoint, JacoPose position)
    {
    	float error = 10.0f; // allow rotation error of 10 degree

    	float positionXDeg = fmod(position.ThetaX * (180.0f / M_PI), 360);
		float positionYDeg = fmod(position.ThetaY * (180.0f / M_PI), 360);
		float positionZDeg = fmod(position.ThetaZ * (180.0f / M_PI), 360);

		if (positionXDeg < 0)
			positionXDeg += 360.0f;

		if (positionYDeg < 0)
				positionYDeg += 360.0f;

		if (positionZDeg < 0)
				positionZDeg += 360.0f;

		float positionTargetXDeg = wayPoint.ThetaX * (180.0f / M_PI);
		float positionTargetYDeg = wayPoint.ThetaY * (180.0f / M_PI);
		float positionTargetZDeg = wayPoint.ThetaZ * (180.0f / M_PI);

		float diffX = fabs(positionXDeg - positionTargetXDeg);

	//	if (diffX > 180.0f)
	//		diffX = 360.0f - diffX;

		float diffY = fabs(positionYDeg - positionTargetYDeg);

	//	if (diffY > 180.0f)
	//		diffY = 360.0f - diffY;

		float diffZ = fabs(positionZDeg - positionTargetZDeg);

	//	if (diffZ > 180.0f)
	//		diffZ = 360.0f - diffZ;

		ROS_INFO_STREAM("Current: " << positionXDeg << ", " << positionYDeg << ", " << positionZDeg);

		if (diffX <= error and
			diffY <= error) // and
		    //diffZ <= error)
			return true;
		else
			return false;
    }

    void GraspingActionServer::checkWaypoint(JacoPose wayPoint)
    {
    	bool reached = false;
    	JacoPose position;

    	ros::Rate rate(20);

    	size_t freezeCounter = 0; // if it has grasped it cannot fully close
		size_t freezeCheck = 50; // if hasn't changed after 20 tries

		JacoPose prevPosition;
		arm.getCartesianPosition(prevPosition);

    	while (!reached)
    	{
    		arm.getCartesianPosition(position);

    	//	if (checkPosition(wayPoint, position) and checkRotation(wayPoint, position))
    	//		reached = true;

    		if (checkPosition(wayPoint, position))
    			reached = true;

    		if (prevPosition.X == position.X and
    			prevPosition.Y == position.Y and
				prevPosition.Z == position.Z and
				prevPosition.ThetaX == position.ThetaX and
				prevPosition.ThetaY == position.ThetaY and
				prevPosition.ThetaZ == position.ThetaZ)
    			++freezeCounter;

    		if (freezeCounter == freezeCheck)
    		{
    			ROS_INFO_STREAM("Movement Froze!!!!!");
    			reached = true;
    		}

    		prevPosition = position;
    		rate.sleep();
    	}
    }


    void GraspingActionServer::checkFingers(FingerAngles fingers)
    {
    	FingerAngles currentFingers;
    	FingerAngles prevFingers;
    	FingerAngles startFingers;

    	arm.getFingerPositions(startFingers);
    	arm.getFingerPositions(prevFingers);
    	float error = 20;

    	bool correct = false;

    	ros::Rate rate(10);

    	size_t freezeCounter = 0; // if it has grasped it cannot fully close
    	size_t freezeCheck = 20; // if hasn't changed after 20 tries

    	while (!correct)
    	{
    		arm.getFingerPositions(currentFingers);

    		if (fabs(currentFingers.Finger1 - fingers.Finger1) < error and
    			fabs(currentFingers.Finger2 - fingers.Finger2) < error)
    			correct = true;

    		if (prevFingers.Finger1 == currentFingers.Finger1 and
    			prevFingers.Finger2 == currentFingers.Finger2)
    		{
    			if (currentFingers.Finger1 == startFingers.Finger1 and
    				currentFingers.Finger2 == startFingers.Finger2)
    				arm.setFingerPositions(fingers, 0, true);

    			++freezeCounter;
    		}

    		if (freezeCounter == freezeCheck)
    			correct = true;

    		prevFingers = currentFingers;

    		ROS_INFO_STREAM("error finger1: " << fabs(currentFingers.Finger1 - fingers.Finger1) << ", finger2: " << fabs(currentFingers.Finger2 - fingers.Finger2));
    		rate.sleep();
    	}
    }

    void GraspingActionServer::movePath(vector<arm_planner::Position> waypoints)
    {
    	bool fingersAreOpen = false;
    	JacoPose wayPoint;

		arm.startAPI();

		for (size_t idx = 0; idx < waypoints.size(); ++idx)
		{
			wayPoint.X = waypoints.at(idx).x;
			wayPoint.Y = waypoints.at(idx).y;
			wayPoint.Z = waypoints.at(idx).z;
			wayPoint.ThetaX = toRad(waypoints.at(idx).thetaX);
			wayPoint.ThetaY = toRad(waypoints.at(idx).thetaY);
			wayPoint.ThetaZ = toRad(waypoints.at(idx).thetaZ);

			ROS_INFO_STREAM("x: " << wayPoint.X << ", y: " << wayPoint.Y << ", z: " << wayPoint.Z);
			ROS_INFO_STREAM("thetaX: " << wayPoint.ThetaX <<", thetaY: " << wayPoint.ThetaY << ", thetaZ: " << wayPoint.ThetaZ);

			if (waypoints.at(idx).approach == true)
			{
				if (waypoints.at(idx).openFingers == true and fingersAreOpen == false)
				{
					fingersAreOpen = true;
					FingerAngles fingers;
					fingers.Finger1 = 0.0;
					fingers.Finger2 = 0.0;
					arm.setFingerPositions(fingers, 0, true);
					checkFingers(fingers);
				}
				else if (waypoints.at(idx).openFingers == true and fingersAreOpen == true)
				{
					arm.sendCartesianPosition(wayPoint, true);
					checkWaypoint(wayPoint);
				}
				else if (waypoints.at(idx).openFingers == false and fingersAreOpen == true)
				{
					fingersAreOpen = false;

					FingerAngles fingers;
					fingers.Finger1= 6400.0;
					fingers.Finger2 = 6400.0;
					arm.setFingerPositions(fingers, 0, true);
					checkFingers(fingers);
				}
				else if (waypoints.at(idx).openFingers == false and fingersAreOpen == false)
				{
					arm.sendCartesianPosition(wayPoint, true);
					checkWaypoint(wayPoint);
				}
			}
			else
			{
				arm.sendCartesianPosition(wayPoint, false);
				checkWaypoint(wayPoint);
			}

		}

	//	arm.stopAPI();
    }

    void GraspingActionServer::executePath(const arm_planner::pathGoalConstPtr &path)
    {
    	ROS_INFO_STREAM("Path size: " << path->waypoints.size());

    	movePath(path->waypoints);

    	if (path->left)
    		d_status = ObjectNavLeft;
    	else if (path->right)
    		d_status = ObjectNavRight;
    }

    void GraspingActionServer::execute(const jaco_msgs::graspingGoalConstPtr &goal)
    {
    	ROS_INFO_STREAM("Starting action grasping");
    	JacoPose position;
	    arm.getCartesianPosition(position);

	    arm.startAPI();
	    bool push = false; 

	    JacoPose holdPosition;
	    holdPosition.X = 0.35;
	    holdPosition.Y = -0.03;
	    holdPosition.Z = 0.40;
	    holdPosition.ThetaX = toRad(90);  // pitch
	    holdPosition.ThetaY = toRad(90);  // yaw
	    holdPosition.ThetaZ = toRad(0.0); // roll

	    if (goal->object.function == "home")
	    {
	    	ROS_INFO_STREAM("Moving to home position");
	    	// move to fixed holding position for driving
	    	if (!arm.setCartesianPosition(holdPosition, 0, push))
	    	{
	    		as.setAborted();
	    		return;
	    	}
	    	else
	    		as.setSucceeded();

	    	return;
	    }
	    else if (goal->object.function == "navigation")
		{
			JacoPose position;

			position.X = 0.30f;
			position.Y = -0.1f;
			position.Z = 0.30f;
			position.ThetaX = toRad(90);
			position.ThetaY = toRad(0.0);
			position.ThetaZ = toRad(0.0);

			arm.sendCartesianPosition(position, false);
			checkWaypoint(position);

			position.X = 0.1f;
			position.Y = 0.15f;
			position.Z = 0.25;
			position.ThetaX = toRad(180);  // pitch
			position.ThetaY = toRad(20);  // yaw
			position.ThetaZ = toRad(0); // roll

			arm.sendCartesianPosition(position, false);
			checkWaypoint(position);

			position.X = 0.0f;
			position.Y = 0.10f;
			position.Z = 0.15f;
			position.ThetaX = toRad(180.0);
			position.ThetaY = toRad(20.0);
			position.ThetaZ = toRad(0.0);

			arm.sendCartesianPosition(position, false);
			checkWaypoint(position);

			d_status = NoObjectNav;

			as.setSucceeded();
			return;

		}
	    else if (goal->object.function == "grasp")
	    {
	    	// grasping motion
	    	JacoPose newPosition;	    
		    newPosition.X = goal->object.x;
		    newPosition.Y = goal->object.y + 0.10f;
		    newPosition.Z = goal->object.z  + 0.10f; // begin 10cm higher of the object so that the closed fingers don't touch the object
		    newPosition.ThetaX = goal->object.thetaX;
		    newPosition.ThetaY = goal->object.thetaY;
		    newPosition.ThetaZ = goal->object.thetaZ; 

		    d_fObjectHeight = goal->object.height; // set the current object height 

		    if (!arm.setCartesianPosition(newPosition, 0, push))
		    {
		    	// error accord, not moving anymore
		    	as.setAborted();
		    	return;
		    }

		    usleep(500000);

		    // open the fingers
		    FingerAngles fingers;
		    fingers.Finger1= 10.0;
		    fingers.Finger2 = 10.0;
		    arm.setFingerPositions(fingers);  // we don't do error checking for opening the fingers, (for now)
		    sleep(1);

			// move down to object height
	    	newPosition.Z -= 0.10f;

	    	if (!arm.setCartesianPosition(newPosition, 0, push, 0.04))
		    {
		    	// error accord, not moving anymore
		    	as.setAborted();
		    	return;
		    }
	    	usleep(100);

	    	 // move forward to the object
	    	newPosition.Y -= 0.10f;

	    	if (!arm.setCartesianPosition(newPosition, 0, push, 0.04))
		    {
		    	// error accord, not moving anymore
		    	as.setAborted();
		    	return;
		    }
		    usleep(500000);

		    // close fingers for grasping object
		    fingers.Finger1= 6400.0;
		    fingers.Finger2 = 6400.0;
		    arm.setFingerPositions(fingers);

		    ros::Rate rate(100);
		    int count = 0;
		    /*
		    AngularPosition fingerForce;

		    while (count < 100)
		    {
				arm.getAngularCurrent(fingerForce);
		//		if (fingerForce.Fingers.Finger1 > 0 || fingerForce.Fingers.Finger2 > 0)
					ROS_INFO_STREAM("Current: " << fingerForce.Fingers.Finger1 << ", " << fingerForce.Fingers.Finger2);
				++count;
				rate.sleep();
		    }
			*/
		    sleep(1);  

		    // TODO
		   	// checking for position of fingers is very dangerous, should perhaps check with torque or amps?
		   	// TODO

		    // move 5cm up with object to clear the table
		    newPosition.Z += 0.05f;
		    if (!arm.setCartesianPosition(newPosition, 0, push))
		    {
		    	// error accord, not moving anymore
		    	as.setAborted();
		    	return;
		    }

		    usleep(100);

		    // move to fixed holding position for driving
		    if (!arm.setCartesianPosition(holdPosition, 0, push))
		    {
		    	// error accord, not moving anymore
		    	as.setAborted();
		    	return;
		    }
		    usleep(100);

		    // assume arm is now in holding position
		    as.setSucceeded();
		    return;

	    }
	    else if (goal->object.function == "handoff") // hand off the object to operator
	    {
	    	if (d_status == ObjectNavLeft)
				moveToLeft(0.5);
			else if (d_status == ObjectNavRight)
				moveToRight(0.5);
			else
				as.setAborted();
	    	// the position of giving the object to the operator
		    JacoPose givePosition;
		    givePosition.X = 0.0;
		    givePosition.Y = -0.40;
		    givePosition.Z = 0.50;
		    givePosition.ThetaX = toRad(90);
		    givePosition.ThetaY = toRad(20);
		    givePosition.ThetaZ = toRad(0.0);

	    	if (!arm.setCartesianPosition(givePosition, 0, push))
		    {
		    	// error accord, not moving anymore
		    	as.setAborted();
		    	return;
		    }
		    sleep(1);

		    // open fingers
		    FingerAngles fingers;
		    fingers.Finger1= 10.0;
		    fingers.Finger2 = 10.0;
		    arm.setFingerPositions(fingers);
		    sleep(3);

		    // close fingers
		    fingers.Finger1= 6000.0;
		    fingers.Finger2 = 6000.0;
		    arm.setFingerPositions(fingers);
		    sleep(1);

		    //CHANGE STUFF HERE!!!]
		    /*
		    // move back to holding position for driving
	    	if (!arm.setCartesianPosition(holdPosition, 0, push))
		    {
		    	// error accord, not moving anymore
		    	as.setAborted();
		    	return;
		    }
	    	usleep(100);
			*/
	    	as.setSucceeded();
	    	return;
	    }
	    else if (goal->object.function == "putDownMOTest") // for the manipulation and object recognition Test (ROBOCUP)
		{
		/*	if (d_status == ObjectNavLeft)
				moveToLeft(goal->object.z);
			else if (d_status == ObjectNavRight)
				moveToRight(goal->object.z);
			else
				as.setAborted();
		*/
	    	arm_planner::Position point;
	    	if (d_status == ObjectNavLeft)
	    	{
	    		point.x = 0.35f;
				point.y = -0.05f;
				point.z = goal->object.z;
				point.thetaX = 90.0f;
				point.thetaY = 20.0f;
				point.thetaZ = 0.0f;
	    	}
			else if (d_status == ObjectNavRight)
			{
				point.x = -0.35f;
				point.y = -0.05f;
				point.z = goal->object.z;
				point.thetaX = 90.0f;
				point.thetaY = -20.0f;
				point.thetaZ = 0.0f;
			}
			else
				as.setAborted();

			vector<arm_planner::Position> vPath;
			vPath.push_back(point);

			arm_planner::Position position;
			position.x = goal->object.x;
			position.y = goal->object.y;
			position.z = goal->object.z + 0.04f; // drop off position +4cm for clearing space below movement

			float angle = -20.0f;

			if (goal->object.x < 0.0)
				angle *= -1;

			position.thetaX = 90.0f;
			position.thetaY = angle;
			position.thetaZ = 0.0f;
			position.approach = false;
			position.openFingers = false;

			vPath.push_back(position);

			position.z = goal->object.z;
			position.approach = true;

			vPath.push_back(position);

			position.openFingers = true;

			vPath.push_back(position);

			float rDistance = 0.11;  // distance in front of object, before approach
			float xOffset = rDistance * sin(angle * M_PI/180);
			float yOffset = rDistance * cos(angle * M_PI/180);

			position.x -= xOffset;
			position.y += yOffset;

			vPath.push_back(position);

			position.z += 0.04f;

			vPath.push_back(position);

			position.openFingers = false;

			vPath.push_back(position);

			position.approach = false;

			vPath.push_back(position);

			position.z += 0.02f;

			vPath.push_back(position);

			// move back to initial position for nav

			position.x = 0.30f;
			position.y = -0.1f;
			position.z = 0.30f;
			position.thetaX = 90;
			position.thetaY = 0.0;
			position.thetaZ = 0.0;

			vPath.push_back(position);

			position.x = 0.1f;
			position.y = 0.15f;
			position.z = 0.25;
			position.thetaX = 180;  // pitch
			position.thetaY = 20;  // yaw
			position.thetaZ = 0; // roll

			vPath.push_back(position);

			position.x = 0.0f;
			position.y = 0.10f;
			position.z = 0.15f;
			position.thetaX = 180.0;
			position.thetaY = 20.0;
			position.thetaZ = 0.0;

			vPath.push_back(position);
			movePath(vPath);
			d_status = NoObjectNav;
		}
	    else if (goal->object.function == "putDown")
		{
	    	if (d_status == ObjectNavLeft)
	    		moveToLeft(goal->object.z);
	    	else if (d_status == ObjectNavRight)
	    		moveToRight(goal->object.z);
	    	else
	    		as.setAborted();

	    	vector<arm_planner::Position> vPath;
	    	arm_planner::Position position;
	    	position.x = goal->object.x;
	    	position.y = goal->object.y;
	    	position.z = goal->object.z + 0.02f; // drop off position +2cm for clearing space below movement

	    	float angle = -20.0f;

	    	if (goal->object.x < 0.0)
				angle *= -1;

	    	position.thetaX = 90.0f;
	    	position.thetaY = angle;
	    	position.thetaZ = 0.0f;
	    	position.approach = false;
			position.openFingers = false;

	    	vPath.push_back(position);

	    	position.z = goal->object.z;
	    	position.approach = true;

	    	vPath.push_back(position);

	    	position.openFingers = true;

	    	vPath.push_back(position);


	    	float rDistance = 0.11;  // distance in front of object, before approach
			float xOffset = rDistance * sin(angle * M_PI/180);
			float yOffset = rDistance * cos(angle * M_PI/180);

			position.x -= xOffset;
			position.y += yOffset;

	    	vPath.push_back(position);

	    	position.z += 0.04f;

			vPath.push_back(position);

	    	position.openFingers = false;

			vPath.push_back(position);

			position.approach = false;

			vPath.push_back(position);

	    	position.z += 0.02f;

	    	vPath.push_back(position);

	    	// move back to initial position for nav

	    	position.x = 0.30f;
			position.y = -0.1f;
			position.z = 0.30f;
			position.thetaX = 90;
			position.thetaY = 0.0;
			position.thetaZ = 0.0;

			vPath.push_back(position);

			position.x = 0.1f;
			position.y = 0.15f;
			position.z = 0.25;
			position.thetaX = 180;  // pitch
			position.thetaY = 20;  // yaw
			position.thetaZ = 0; // roll

			vPath.push_back(position);

			position.x = 0.0f;
			position.y = 0.10f;
			position.z = 0.15f;
			position.thetaX = 180.0;
			position.thetaY = 20.0;
			position.thetaZ = 0.0;

			vPath.push_back(position);
			movePath(vPath);
			d_status = NoObjectNav;


	    	/*
			JacoPose newPosition;
			newPosition.X = goal->object.x;  // either 0 or most right position
			newPosition.Y = goal->object.y;	 // if always the same distance 0.5f
			newPosition.Z = goal->object.z  + d_fObjectHeight - 0.02f + 0.10f; // height of table + grasping height + 10cm
			newPosition.ThetaX = goal->object.thetaX; // always the same orientation
			newPosition.ThetaY = goal->object.thetaY;
			newPosition.ThetaZ = goal->object.thetaZ;

			// go to object location but with height of table
			if (!arm.setCartesianPosition(newPosition, true, 0, push))
			{
				// error accord, not moving anymore
				as.setAborted();
				return;
			}

			// move down the 10cm
			newPosition.Z -= 0.10f;

			if (!arm.setCartesianPosition(newPosition, true, 0, push, 0.04)) // move down slowly
			{
				// error accord, not moving anymore
				as.setAborted();
				return;
			}

			usleep(500000);
			// open fingers
			FingerAngles fingers;
			fingers.Finger1= 10.0;
			fingers.Finger2 = 10.0;
			arm.setFingerPositions(fingers);
			sleep(1);

			newPosition.Y += 0.05f;

			if (!arm.setCartesianPosition(newPosition, true, 0, push, 0.08))
			{
				// error accord, not moving anymore
				as.setAborted();
				return;
			}

			newPosition.Z += 0.20f;

			if (!arm.setCartesianPosition(newPosition, true, 0, push, 0.10))
			{
				// error accord, not moving anymore
				as.setAborted();
				return;
			}

			usleep(500000);
			// close fingers
			fingers.Finger1= 6000.0;
			fingers.Finger2 = 6000.0;
			arm.setFingerPositions(fingers);
			sleep(1);

			// move to fixed holding position for driving
			if (!arm.setCartesianPosition(holdPosition, true, 0, push))
			{
				// error accord, not moving anymore
				as.setAborted();
				return;
			}
			usleep(100);

			as.setSucceeded();
			return;

			*/
		}
	    else if (goal->object.function == "drop") // for pooring out the cup in a container of some sort
	    {
	    	JacoPose newPosition;	    
		    newPosition.X = goal->object.x;
		    newPosition.Y = goal->object.y;
		    newPosition.Z = goal->object.z  + 0.13f; 
		    newPosition.ThetaX = goal->object.thetaX;
		    newPosition.ThetaY = goal->object.thetaY;
		    newPosition.ThetaZ = goal->object.thetaZ;  

		    if (!arm.setCartesianPosition(newPosition, 0, push))
		    {
		    	// error accord, not moving anymore
		    	as.setAborted();
		    	return;
		    }

		 //   position.Z -= 0.05f;
		 //   position.X += 0.03f;
		    newPosition.ThetaZ = -toRad(170); // roll

		    if (!arm.setCartesianPosition(newPosition, 0, push, 0.1, 0.90))
		    {
		    	// error accord, not moving anymore
		    	as.setAborted();
		    	return;
		    }

		    newPosition.ThetaZ = -toRad(0); // roll

		    if (!arm.setCartesianPosition(newPosition, 0, push, 0.1, 0.90))
		    {
		    	// error accord, not moving anymore
		    	as.setAborted();
		    	return;
		    }

		    newPosition.X = goal->object.xLeft + 0.15f; // move 15cm to the left of bin,  from middel of hand to open finger is 10cm

		    if (!arm.setCartesianPosition(newPosition, 0, push))
		    {
		    	// error accord, not moving anymore
		    	as.setAborted();
		    	return;
		    }

		    // put the object back on the table
		    float zGoal = goal->object.tableHeight + d_fObjectHeight - 0.02f;

		    newPosition.Z = zGoal;

		    if (!arm.setCartesianPosition(newPosition, 0, push, 0.04))
		    {
		    	// error accord, not moving anymore
		    	as.setAborted();
		    	return;
		    }

			usleep(500000);
		    // open fingers
		    FingerAngles fingers;
		    fingers.Finger1= 10.0;
		    fingers.Finger2 = 10.0;
		    arm.setFingerPositions(fingers);
		    sleep(1);

		    newPosition.Y += 0.05f;

		    if (!arm.setCartesianPosition(newPosition, 0, push, 0.08))
		    {
		    	// error accord, not moving anymore
		    	as.setAborted();
		    	return;
		    }

		    newPosition.Z += 0.20f;

		    if (!arm.setCartesianPosition(newPosition, 0, push, 0.10))
		    {
		    	// error accord, not moving anymore
		    	as.setAborted();
		    	return;
		    }

		    usleep(500000);
		    // close fingers
		    fingers.Finger1= 6000.0;
		    fingers.Finger2 = 6000.0;
		    arm.setFingerPositions(fingers);
		    sleep(1);

		    // move to fixed holding position for driving
		    if (!arm.setCartesianPosition(holdPosition, 0, push))
		    {
		    	// error accord, not moving anymore
		    	as.setAborted();
		    	return;
		    }
		    usleep(100);

		    as.setSucceeded();
		    return;
	    }

	    as.setAborted();
    }
}
