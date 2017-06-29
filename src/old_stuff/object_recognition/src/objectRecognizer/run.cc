#include "objectRecognizer.ih"

/*
Contains: 
rosrun: complete run of classifying objects in view
run: complete run, given no camera input, images from file
recordExperiment: complete run for recording an experiment
runExperiment: load images, then run experiment from files

Receives camera input or reads input files, passes images to 
processing (processPCL) and calls the classifier
*/


void ObjectRecognizer::rosrun(const sensor_msgs::PointCloud2ConstPtr &inputMsg)
{
	if (!d_pause)
	{
		cout << "\nstarting new run!" << endl;

		tfTransformer->waitForTransform("/mico_base_link", "/front_xtion_link", ros::Time::now(), ros::Duration(1.0));

		if (!tfTransformer->canTransform("/mico_base_link", inputMsg->header.frame_id, inputMsg->header.stamp))
		{
			ROS_INFO_STREAM_ONCE("No transform found");
			return;
		}
		else
		{
			ROS_INFO_STREAM_ONCE("Transform is found again");
		}

		sensor_msgs::PointCloud2 cloudMsg_transformed;
		pcl_ros::transformPointCloud("/mico_base_link", *inputMsg, cloudMsg_transformed, *tfTransformer);

		PCLPointCloudPtr PCLimg(new PCLPointCloud);
		fromROSMsg(cloudMsg_transformed, *PCLimg); // convertion from ROS to PCL formats

		processPCL(PCLimg);

		classify();
	}
	jsonParser(d_pause);
}

void ObjectRecognizer::run(PCLPointCloudPtr inputImg)
{
	processPCL(inputImg);

	classify();
}

void ObjectRecognizer::recordExperiment(const sensor_msgs::PointCloud2ConstPtr &inputMsg)
{
	if (objectName == "")
	{
		cout << "what object are you recording?" << endl;
		cin >> objectName;
		cin.ignore();
	}

	cout << "\nstarting new run!" << endl;

	PCLPointCloudPtr PCLimg(new PCLPointCloud); 
	fromROSMsg(*inputMsg, *PCLimg); // convertion from ROS to PCL formats

	string numstr;          // string which will contain the result
	ostringstream convert;   // stream used for the conversion
	convert << numFrames % 90;      // insert the textual representation of 'Number' in the characters in the stream
	numstr = convert.str(); // set 'Result' to the contents of the stream	string filename = "experiments/" + objectName + "/image" + numstr;
	string filename = "experiments/" + objectName + "/image" + numstr;
//	io::savePCDFile(filename, *PCLimg);

	
	processPCL(inputMsg);

	classify();

	++numFrames;
	cout << "processed frame " << numFrames % 30 << "/30" << endl;

	//If a given orientation has been recorder 30 times, wait
	//for the user to turn the object. If recording is over,
	//write the results and wait for new object
	if (numFrames % 30 == 0)
	{
		++orientation;
		if (orientation % 3 == 0)
		{
			writeResults("experiments/" + objectName);
			cout << "This object has been completely recorded, please place a new object if you wish. If you do so, what is the new object called?" << endl;
			cin >> objectName;
			cin.ignore();
		} else
		{
			cout << "recorded 30 samples, please turn object. Press enter when ready to continue" << endl;
			cin.ignore();
		}
	}
}

void ObjectRecognizer::runExperiment()
{
	//Read the imput images for the experiment. For each image,
	//make a complete run of classifying objects in view and write
	//the results afterwards
	cout << "running experiment" << endl;
	string location = "experiments";

    DIR *dir;
    struct dirent *folders;

    if((dir  = opendir(location.c_str() ) ) == NULL) 
    {
        cout << "Error(" << errno << ") opening " << location << endl;
        return;
    }

    while ((folders = readdir(dir)) != NULL) 
    {
        if (folders->d_name[0] == '.')
            continue;

	    string classLocation = location + "/" + folders->d_name;

	    DIR *classdir;
        struct dirent *files;
        if((classdir  = opendir(classLocation.c_str() ) ) == NULL) 
        {
            cout << "Error(" << errno << ") opening " << classLocation << endl;
            return;
        }

        cout << "testing the " << folders->d_name << " class" << endl;
        while ((files = readdir(classdir)) != NULL) 
        {
            if (files->d_name[0] != 'i')
                continue;

            PointCloud<Point>::Ptr cloud (new pcl::PointCloud<Point>);

            if (io::loadPCDFile<Point> (classLocation + "/" + files->d_name, *cloud) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read %s \n", files->d_name);
                continue;
            }

            run(cloud);
        }

        cout << "writing results" << endl;
        writeResults(classLocation);
    }
}

