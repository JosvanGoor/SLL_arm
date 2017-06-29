#include "moveit_interface.ih"

void MoveItInterface::readFromMemory()
{
	readEmergencyStateFromMemory();
	readXYZQuaternionGoalFromMemory();
	readXYZRPYGoalFromMemory();
	readDangerousXYZQuaternionGoalFromMemory();
	readDangerousJointAnglesGoalFromMemory();
	readPredefinedGoalFromMemory();
	readFingerGoalFromMemory();
	readSimpleGrabFromMemory();
	readCancelGoalFromMemory();
}

void MoveItInterface::readEmergencyStateFromMemory()
{
	// Create read message
	borg_pioneer::MemoryReadSrv srv;
	srv.request.function = "get_last_observation";
	srv.request.timestamp = ros::Time::now();
	srv.request.name = "EMERGENCY";
	srv.request.params = "";

	// Make the call
	if (!d_memory_reader.call(srv))
		return;

	// Read message
	std::string msg = srv.response.json;
	if (msg.compare("null") == 0)
		return;

	// Parse it
	Json::Value root;
	Json::Reader reader;
	bool parsedSuccess = reader.parse(msg, root, false);

	// Check if parsing is successful
	if (!parsedSuccess)
	{
		ROS_WARN_STREAM("Failed to parse JSON\n" << reader.getFormatedErrorMessages() << '\n');
		return;
	}

	//Extracting time
	Json::Value const time = root["time"];
	if (!time.isNull())
		d_last_time_memory_emergency_received = ros::Time(time.asDouble());
}

void MoveItInterface::readXYZQuaternionGoalFromMemory()
{
	// Create read message
	borg_pioneer::MemoryReadSrv srv;
	srv.request.function = "get_last_observation";
	srv.request.timestamp = ros::Time::now();
	srv.request.name = "moveit_XYZ_quaternion_goal";
	srv.request.params = "";

	// Make the call
	if (!d_memory_reader.call(srv))
		return;

	// Read message
	std::string msg = srv.response.json;
	if (msg.compare("null") == 0)
		return;

	// Parse it
	Json::Value root;
	Json::Reader reader;
	bool parsedSuccess = reader.parse(msg, root, false);

	// Check if parsing is successful
	if (!parsedSuccess)
	{
		ROS_WARN_STREAM("Failed to parse JSON\n" << reader.getFormatedErrorMessages() << '\n');
		return;
	}

	try
	{
		// Extracting time
		Json::Value const time = root["time"];
		Json::Value const goal_position = root["position"];
		Json::Value const goal_orientation = root["orientation"];

		// Extract coordinates	
		double goal_position_x = goal_position["x"].asDouble();
		double goal_position_y = goal_position["y"].asDouble();
		double goal_position_z = goal_position["z"].asDouble();
		double goal_orientation_x = goal_orientation["xx"].asDouble();
		double goal_orientation_y = goal_orientation["yy"].asDouble();
		double goal_orientation_z = goal_orientation["zz"].asDouble();
		double goal_orientation_w = goal_orientation["w"].asDouble();

		// Check data and give goal
		if (!time.isNull() && !goal_position.isNull() && !goal_orientation.isNull())
		{
			ros::Duration d = ros::Time(time.asDouble()) - d_last_time_memory_goal_received;
			if (d.toSec() > 0 && !d_is_planning)
			{
				d_last_time_memory_goal_received = ros::Time(time.asDouble());
				planXYZQuaternionGoal(goal_position_x, goal_position_y, goal_position_z, goal_orientation_x, goal_orientation_y, goal_orientation_z, goal_orientation_w);
			}
		}
	}
	catch ( ... )
	{
		ROS_INFO("Cannot interpret provided XYZ-quaternion goal");
	}
}

void MoveItInterface::readXYZRPYGoalFromMemory()
{
	// Create read message
	borg_pioneer::MemoryReadSrv srv;
	srv.request.function = "get_last_observation";
	srv.request.timestamp = ros::Time::now();
	srv.request.name = "moveit_XYZ_RPY_goal";
	srv.request.params = "";

	// Make the call
	if (!d_memory_reader.call(srv))
		return;

	// Read message
	std::string msg = srv.response.json;
	if (msg.compare("null") == 0)
		return;

	// Parse it
	Json::Value root;
	Json::Reader reader;
	bool parsedSuccess = reader.parse(msg, root, false);

	// Check if parsing is successful
	if (!parsedSuccess)
	{
		ROS_WARN_STREAM("Failed to parse JSON\n" << reader.getFormatedErrorMessages() << '\n');
		return;
	}

	try
	{
		// Extracting time
		Json::Value const time = root["time"];
		Json::Value const goal_position = root["position"];
		Json::Value const goal_orientation = root["orientation"];

		// Extract coordinates	
		double goal_position_x = goal_position["x"].asDouble();
		double goal_position_y = goal_position["y"].asDouble();
		double goal_position_z = goal_position["z"].asDouble();
		double goal_orientation_r = goal_orientation["r"].asDouble();
		double goal_orientation_p = goal_orientation["p"].asDouble();
		double goal_orientation_y = goal_orientation["yy"].asDouble();

	//	ROS_INFO_STREAM("GOAL POSITION, POSE");
	//	ROS_INFO_STREAM("GOAL: " << goal_position_x << ", " << goal_position_y << ", " << goal_position_z);
	//	ROS_INFO_STREAM("POSE: " << goal_orientation_r << ", " << goal_orientation_p << ", "<< goal_orientation_y);

		// Check data and give goal
		if (!time.isNull() && !goal_position.isNull() && !goal_orientation.isNull())
		{
			ros::Duration d = ros::Time(time.asDouble()) - d_last_time_memory_goal_received;
			if (d.toSec() > 0 && !d_is_planning)
			{
				d_last_time_memory_goal_received = ros::Time(time.asDouble());
				planXYZRPYGoal(goal_position_x, goal_position_y, goal_position_z, goal_orientation_r, goal_orientation_p, goal_orientation_y);
			}
		}
	}
	catch ( ... )
	{
		ROS_INFO("Cannot interpret provided XYZ-RPY goal");
	}
}

void MoveItInterface::readDangerousXYZQuaternionGoalFromMemory()
{
	// Create read message
	borg_pioneer::MemoryReadSrv srv;
	srv.request.function = "get_last_observation";
	srv.request.timestamp = ros::Time::now();
	srv.request.name = "moveit_dangerous_XYZ_quaternion_goal";
	srv.request.params = "";

	// Make the call
	if (!d_memory_reader.call(srv))
		return;

	// Read message
	std::string msg = srv.response.json;
	if (msg.compare("null") == 0)
		return;

	// Parse it
	Json::Value root;
	Json::Reader reader;
	bool parsedSuccess = reader.parse(msg, root, false);

	// Check if parsing is successful
	if (!parsedSuccess)
	{
		ROS_WARN_STREAM("Failed to parse JSON\n" << reader.getFormatedErrorMessages() << '\n');
		return;
	}

	try
	{
		// Extracting time
		Json::Value const time = root["time"];
		Json::Value const goal_position = root["position"];
		Json::Value const goal_orientation = root["orientation"];

		// Extract coordinates	
		double goal_position_x = goal_position["x"].asDouble();
		double goal_position_y = goal_position["y"].asDouble();
		double goal_position_z = goal_position["z"].asDouble();
		double goal_orientation_x = goal_orientation["x"].asDouble();
		double goal_orientation_y = goal_orientation["y"].asDouble();
		double goal_orientation_z = goal_orientation["z"].asDouble();
		double goal_orientation_w = goal_orientation["w"].asDouble();

		// Check data and give goal
		if (!time.isNull() && !goal_position.isNull() && !goal_orientation.isNull())
		{
			ros::Duration d = ros::Time(time.asDouble()) - d_last_time_memory_goal_received;
			if (d.toSec() > 0 && !d_is_planning)
			{
				d_last_time_memory_goal_received = ros::Time(time.asDouble());
				planDangerousXYZQuaternionGoal(goal_position_x, goal_position_y, goal_position_z, goal_orientation_x, goal_orientation_y, goal_orientation_z, goal_orientation_w);
			}
		}
	}
	catch ( ... )
	{
		ROS_INFO("Cannot interpret provided XYZ-quaternion goal");
	}
}

void MoveItInterface::readDangerousJointAnglesGoalFromMemory()
{
	// Create read message
	borg_pioneer::MemoryReadSrv srv;
	srv.request.function = "get_last_observation";
	srv.request.timestamp = ros::Time::now();
	srv.request.name = "moveit_dangerous_joint_angles_goal";
	srv.request.params = "";

	// Make the call
	if (!d_memory_reader.call(srv))
		return;

	// Read message
	std::string msg = srv.response.json;
	if (msg.compare("null") == 0)
		return;

	// Parse it
	Json::Value root;
	Json::Reader reader;
	bool parsedSuccess = reader.parse(msg, root, false);

	// Check if parsing is successful
	if (!parsedSuccess)
	{
		ROS_WARN_STREAM("Failed to parse JSON\n" << reader.getFormatedErrorMessages() << '\n');
		return;
	}

	try
	{
		// Extracting time
		Json::Value const time = root["time"];
		Json::Value const goal_joint_angles = root["angles"];

		// Extract coordinates
		std::vector<double> joint_angles;
		for (int i = 0; i < 6; ++i)
			joint_angles.push_back(goal_joint_angles[i].asDouble());

		// Check data and give goal
		if (!time.isNull() && !goal_joint_angles.isNull())
		{
			ros::Duration d = ros::Time(time.asDouble()) - d_last_time_memory_goal_received;
			if (d.toSec() > 0 && !d_is_planning)
			{
				d_last_time_memory_goal_received = ros::Time(time.asDouble());
				planDangerousJointAnglesGoal(joint_angles);
			}
		}
	}
	catch ( ... )
	{
		ROS_INFO("Cannot interpret provided joint angles goal");
	}
}

void MoveItInterface::readPredefinedGoalFromMemory()
{
	// Create read message
	borg_pioneer::MemoryReadSrv srv;
	srv.request.function = "get_last_observation";
	srv.request.timestamp = ros::Time::now();
	srv.request.name = "moveit_predefined_goal";
	srv.request.params = "";

	// Make the call
	if (!d_memory_reader.call(srv))
		return;

	// Read message
	std::string msg = srv.response.json;
	if (msg.compare("null") == 0)
		return;

	// Parse it
	Json::Value root;
	Json::Reader reader;
	bool parsedSuccess = reader.parse(msg, root, false);

	// Check if parsing is successful
	if (!parsedSuccess)
	{
		ROS_WARN_STREAM("Failed to parse JSON\n" << reader.getFormatedErrorMessages() << '\n');
		return;
	}

	//Extracting time
	Json::Value const time = root["time"];
	Json::Value const goal_name = root["goal"];

	// Check data and give goal
	if (!time.isNull() && !goal_name.isNull())
	{
		ros::Duration d = ros::Time(time.asDouble()) - d_last_time_memory_goal_received;
		if (d.toSec() > 0 && !d_is_planning)
		{
			d_last_time_memory_goal_received = ros::Time(time.asDouble());
			std::string goal_name_str = goal_name.asString();
			planPredefinedGoal(goal_name_str);
		}
	}
}

void MoveItInterface::readFingerGoalFromMemory()
{
	// Create read message
	borg_pioneer::MemoryReadSrv srv;
	srv.request.function = "get_last_observation";
	srv.request.timestamp = ros::Time::now();
	srv.request.name = "moveit_finger_goal";
	srv.request.params = "";

	// Make the call
	if (!d_memory_reader.call(srv))
		return;

	// Read message
	std::string msg = srv.response.json;
	if (msg.compare("null") == 0)
		return;

	// Parse it
	Json::Value root;
	Json::Reader reader;
	bool parsedSuccess = reader.parse(msg, root, false);

	// Check if parsing is successful
	if (!parsedSuccess)
	{
		ROS_WARN_STREAM("Failed to parse JSON\n" << reader.getFormatedErrorMessages() << '\n');
		return;
	}

	//Extracting time
	Json::Value const time = root["time"];
	Json::Value const goal_name = root["goal"];

	// Check data and give goal
	if (!time.isNull() && !goal_name.isNull())
	{
		ros::Duration d = ros::Time(time.asDouble()) - d_last_time_memory_goal_received;
		if (d.toSec() > 0 && !d_is_planning)
		{
			d_last_time_memory_goal_received = ros::Time(time.asDouble());
			if (goal_name.isString())
			{
				std::string goal_name_str = goal_name.asString();
				std::transform(goal_name_str.begin(), goal_name_str.end(), goal_name_str.begin(), ::tolower);
				if (goal_name_str == "open")
					openFingers();
				else if (goal_name_str == "close")
					closeFingers();
				else
					ROS_WARN_STREAM("Unknown finger position: " << goal_name_str);
			}
			else if (goal_name.isDouble() || goal_name.isInt())
			{
				// Convert to int instead
				double goal_pos = goal_name.asDouble();
				if (goal_pos < 0 || goal_pos > 6000)
				{
					ROS_WARN_STREAM("Finger position out of bounds: " << goal_pos << ". Should be between 0 (open) and 6000 (close)");
					return;
				}
				else
					customFingerPosition((int)goal_pos);
			}
			else
				ROS_WARN_STREAM("Cannot interpret provided finger position");
		}
	}
}

void MoveItInterface::readCancelGoalFromMemory()
{
	// Create read message
	borg_pioneer::MemoryReadSrv srv;
	srv.request.function = "get_last_observation";
	srv.request.timestamp = ros::Time::now();
	srv.request.name = "moveit_cancel_goal";
	srv.request.params = "";

	// Make the call
	if (!d_memory_reader.call(srv))
		return;

	// Read message
	std::string msg = srv.response.json;
	if (msg.compare("null") == 0)
		return;

	// Parse it
	Json::Value root;
	Json::Reader reader;
	bool parsedSuccess = reader.parse(msg, root, false);

	// Check if parsing is successful
	if (!parsedSuccess)
	{
		ROS_WARN_STREAM("Failed to parse JSON\n" << reader.getFormatedErrorMessages() << '\n');
		return;
	}

	//Extracting time
	Json::Value const time = root["time"];
	if (!time.isNull())
	{
		ros::Duration d = ros::Time(time.asDouble()) - d_last_time_memory_cancel_goal_received;
		if (d.toSec() > 0)
		{
			d_last_time_memory_cancel_goal_received = ros::Time(time.asDouble());
			
			// Cancel goals
			cancelGoal();
		}
	}
}

void MoveItInterface::readSimpleGrabFromMemory()
{
	// Create read message
	borg_pioneer::MemoryReadSrv srv;
	srv.request.function = "get_last_observation";
	srv.request.timestamp = ros::Time::now();
	srv.request.name = "moveit_simple_grab";
	srv.request.params = "";

	// Make the call
	if (!d_memory_reader.call(srv))
		return;

	// Read message
	std::string msg = srv.response.json;
	if (msg.compare("null") == 0)
		return;

	// Parse it
	Json::Value root;
	Json::Reader reader;
	bool parsedSuccess = reader.parse(msg, root, false);

	// Check if parsing is successful
	if (!parsedSuccess)
	{
		ROS_WARN_STREAM("Failed to parse JSON\n" << reader.getFormatedErrorMessages() << '\n');
		return;
	}

	//Extracting time
	Json::Value const time = root["time"];
	Json::Value const goal_position = root["grabControl"];


	// Check data and give goal
	if (!time.isNull() && !goal_position.isNull())
	{
		ros::Duration d = ros::Time(time.asDouble()) - d_last_time_memory_goal_received;
		if (d.toSec() > 0 && !d_is_planning)
		{
			d_last_time_memory_goal_received = ros::Time(time.asDouble());
			// Extract coordinates	
			double goal_position_x = goal_position["x"].asDouble();
			double goal_position_y = goal_position["y"].asDouble();
			double goal_position_z = goal_position["z"].asDouble();
			
			simpleMoveGrab(goal_position_x, goal_position_y, goal_position_z);
		}
	}

}

void MoveItInterface::sendFeedbackToMemory(bool success)
{
	// Make message
	borg_pioneer::MemorySrv srv;
	srv.request.timestamp = ros::Time::now();
	srv.request.name = "moveit_feedback";
	char jsonmsg[255];
	sprintf(jsonmsg, "{\"success\": %d}", success);
	srv.request.json = std::string(jsonmsg);

	// Call
	d_memory_writer.call(srv);
}

void MoveItInterface::sendGoalFeedbackToMemory()
{
	sendFeedbackToMemory(d_execution_result == 0);
}

void MoveItInterface::sendDangerousGoalFeedbackToMemory()
{
	sendFeedbackToMemory(d_dangerous_execution_result == 3);
}

void MoveItInterface::sendFingerFeedbackToMemory()
{
	sendFeedbackToMemory(d_finger_result == 3);
}