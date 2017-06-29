#include "objectRecognizer.ih"

bool ObjectRecognizer::jsonParser(bool &type)
{
	borg_pioneer::MemoryReadSrv srv;
	srv.request.function = "get_last_observation";
	srv.request.timestamp = ros::Time::now();
	srv.request.name = "pause_command";
	srv.request.params = "";

	// Make the call
	if (not d_clientReader.call(srv))
	{

		ROS_WARN_ONCE("No connection\n");
		return false;
	}
	// Read message
	string msg = srv.response.json;

	// Let's parse it
	Json::Value root;
	Json::Reader reader;
	bool parsedSuccess = reader.parse(msg, root, false);

	if (not parsedSuccess)
	{
		// Report failures and their locations
		// in the document.
		ROS_ERROR_STREAM("Failed to parse JSON\n" << reader.getFormatedErrorMessages() << '\n');
		return false;
	}

	//Extracting Parts
	Json::Value const time = root["time"];
	double cmd_time;
	string cmd;

	if (not time.isNull())
	{
		cmd_time = time.asDouble();

		//Only continue if the command is new
		if (cmd_time <= d_lastCommand)
			return false;
		d_lastCommand = cmd_time;
	}

	Json::Value const command = root["pause_status"];

	if (not command.isNull())
	{
		type = command.asBool();
	}

}