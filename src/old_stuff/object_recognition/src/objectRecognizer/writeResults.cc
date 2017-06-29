#include "objectRecognizer.ih"

/*
For an item in an experiment write a resultfile (results.txt), which includes
how many times the object was recognized as which object and the runtime of
several steps of the process (e.g. clustering, sampling), and place it in the
folder corresponding to the given item (passed as a string 'location')
*/

void ObjectRecognizer::writeResults(string location)
{
	string resultsfile = location + "/results.txt";
	ofstream schrijf(resultsfile.c_str());

	//Write how many times the object has been reconized as what
	for (map<string, size_t>::iterator resit = results.begin(); resit != results.end(); ++resit)
	{
		schrijf << resit->first << ": " << resit->second << endl;
		resit->second = 0;
	}

	//Calculate runtime of the recognition, including the runtime of 5 substeps:
	//Clustering, cleaning up, sampling, calculating descriptors and classification
	double mintime = DBL_MAX;
	double maxtime = 0.0;
	double totaltime = 0.0;

	double clustertime = 0.0;
	double cleanuptime = 0.0;
	double samplingtime = 0.0;
	double descriptortime = 0.0;
	double classifytime = 0.0;

	for (size_t idx = 0; idx != runtimes.size(); ++idx)
	{
		totaltime += runtimes[idx];
		clustertime += clustertimes[idx];
		cleanuptime += cleanuptimes[idx];
		samplingtime += samplingtimes[idx];
		descriptortime += descriptortimes[idx];
		classifytime += classifytimes[idx];

		if (runtimes[idx] > maxtime)
			maxtime = runtimes[idx];

		if (runtimes[idx] < mintime)
			mintime = runtimes[idx];
	}

	//Write the different runtimes to 'results.txt'
	schrijf << "minimal runtime: " << mintime << endl;
	schrijf << "maximal runtime: " << maxtime << endl;
	schrijf << "average runtime: " << totaltime / runtimes.size() << endl;

  	schrijf << "clustertimes: " << clustertime / runtimes.size() << endl;
  	schrijf << "cleanuptimes: " << cleanuptime / runtimes.size() << endl;
  	schrijf << "samplingtimes: " << samplingtime / runtimes.size() << endl;
  	schrijf << "descriptortimes: " << descriptortime / runtimes.size() << endl;
  	schrijf << "classifytimes: " << classifytime / runtimes.size() << endl;

  	//Clean the datamembers, so a new experiment with a new object can start 
	runtimes.clear();
	clustertimes.clear();
	cleanuptimes.clear();
	samplingtimes.clear();
	descriptortimes.clear();
	classifytimes.clear();


	schrijf.close();
}