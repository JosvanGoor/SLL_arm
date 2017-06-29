#include "objectDetection.h"

void ObjectDetection::writeStop()
{
	borg_pioneer::MemorySrv srv;
	srv.request.timestamp = ros::Time::now();
	srv.request.name = "stop_detection";
	stringstream jsonmsg;

	jsonmsg << "{\"stopped_detection\" : \"true\"}";
	srv.request.json = jsonmsg.str();
	client_writer.call(srv);

	d_currentSampleNr = 0;
}