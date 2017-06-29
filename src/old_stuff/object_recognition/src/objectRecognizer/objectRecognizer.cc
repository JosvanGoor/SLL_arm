#include "objectRecognizer.ih"

/*
Constructor of the recognizer. Sets a few variables, initializes publishers and loads the object database
*/


ObjectRecognizer::ObjectRecognizer():
d_threshold(0.4),
d_publish(true),
numFrames(0),
orientation(0),
objectName(""),
d_pause(true),
d_lastCommand(0)
{
	d_initRos = isInitialized();

	if (d_initRos)
	{
	    d_pubClean = d_rosHandle.advertise<PCLPointCloud>("clean_object_clusters", 1);
	    d_pubAll = d_rosHandle.advertise<PCLPointCloud>("all_object_clusters", 1);
	    d_pubSampled = d_rosHandle.advertise<PCLPointCloud>("sampled_object_clusters", 1);
	    d_clientWriter = d_rosHandle.serviceClient<borg_pioneer::MemorySrv>("memory");
	    d_clientReader = d_rosHandle.serviceClient<borg_pioneer::MemoryReadSrv>("memory_read");
	}

	loadDB();

	cout << "Initialization finished" << endl;

}
