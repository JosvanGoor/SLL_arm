#include "objectDetection.h"

bool ObjectDetection::run()
{
	// read from memory if processing for object detection needs to take place
	// Create request
	borg_pioneer::MemoryReadSrv srv;
	srv.request.timestamp = ros::Time::now();
	srv.request.function = "get_last_observation";
	srv.request.params = "";
	srv.request.name = "perform_object_detection";

	// Make the call
	client_reader.call(srv);

	// Read message
	string message = srv.response.json;
	
	// Check if there is any data
	if (message.size() == 0 || strcmp(message.c_str(), "null") == 0)
	{
		return false;
	}

	if (message.size() > 3)
	{
		//Get rid of unwanted info
		message = message.substr(1, message.size() - 2);		// Remove outer brackets
		size_t first_occurrence_opening_bracket = message.find_first_of('{');
		size_t last_occurrence_closing_bracket = message.find_last_of('}');
		message = message.substr(first_occurrence_opening_bracket, last_occurrence_closing_bracket - first_occurrence_opening_bracket + 1);

		// Separate responses, message can contain multiple responses
		std::vector<std::string> responses;
		boost::char_separator<char> sep("}");
		boost::tokenizer< boost::char_separator<char> > tokens(message, sep);
		int count = 0;
		for (boost::tokenizer<boost::char_separator<char> >::iterator it = tokens.begin(); it != tokens.end(); ++it)
		{
			std::string single_response = *it;
			single_response += '}';	// Add the closing bracket that was first removed by splitting the response
			if (count != 0) single_response = single_response.substr(2, single_response.size() - 2);	// Remove seperating comma between single responses
			responses.push_back(single_response);
			++count;
		}

		size_t countt = 0;
		// Decode data
		foreach (std::string response, responses)
		{
			ptree pt;
			std::stringstream ss; 
			ss << response;
			boost::property_tree::read_json(ss, pt);

			if (countt == 0)
			{
				d_numberOfSamples = pt.get<size_t>("nrOfSamples");
				++countt;
			}
			else
				return pt.get<bool>("perform");
		}

		return false;
	}
	else
		return false;
/*
	// Check if there is any data
	if (message.size() == 0 || strcmp(message.c_str(), "null") == 0)
	{
		// No data, no processing needed
		return false;
	}
	else
		return true;	
*/
}