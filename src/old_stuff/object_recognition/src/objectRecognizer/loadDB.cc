#include "objectRecognizer.ih"
#include <time.h>

/*
Contains: 
loadDB: loads all files from all folders from the 'data' folder to shot- and dimDatabase
getInstances: loads all files from specific item folder

Given a data folder that contains a folder for each item that is to be loaded into the database,
loop over each folder, fill the SHOTdatabase with the SHOTdata and fill the dimension-
database with the max and min values of each dimension
*/

void ObjectRecognizer::loadDB(string DBlocation)
{
    DIR *dir;
    struct dirent *folders;
    vector<double> tmpDimensions;
    
    //Check whether the target folder can be opened
    if((dir  = opendir(DBlocation.c_str() ) ) == NULL) 
    {
        cout << "Error(" << errno << ") opening " << DBlocation << endl;
        return;
    }

    //for all item folders, open them and extract the data
    while ((folders = readdir(dir)) != NULL) 
    {
        if (folders->d_name[0] == '.')
            continue;
        cout << "loading the " << folders->d_name << " class" << endl;
        tmpDimensions.clear();
        tmpDimensions.resize(6);

        d_shotDatabase[folders->d_name] = getInstances(DBlocation + "/" + folders->d_name, tmpDimensions);
        d_dimDatabase[folders->d_name] = tmpDimensions;

        results[folders->d_name] = 0;
    }

    results["nothing"] = 0;
}

shotData ObjectRecognizer::getInstances(string path, vector<double> &tmpDimensions)
{
    DIR *dir;
    struct dirent *files;

    shotData instances;
    
    vector<double> xs;
    vector<double> ys;
    vector<double> zs;

    //Check whether the target folder can be opened
    if((dir  = opendir(path.c_str() ) ) == NULL) 
    {
        cout << "Error(" << errno << ") opening " << path << endl;
        return instances;
    }

    vector<size_t> sizes;

    //for all files, check if it is a SHOT file, and if so, store data in SHOTdatabase
    while ((files = readdir(dir)) != NULL) 
    {
        //All SHOTdata files are named 'file#.pcd'
        if (files->d_name[0] != 'f')
            continue;

        PointCloud<descriptorType>::Ptr cloud (new pcl::PointCloud<descriptorType>);

        if (io::loadPCDFile<descriptorType> (path + "/" + files->d_name, *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read %s \n", files->d_name);
            continue;
        }

        instances.push_back(cloud);//tmpFeature);
        sizes.push_back(cloud->points.size());

    }

    //Store dimensiondata in dimDatabase
    double stdDevSize;
    accumulator_set<double, stats<tag::variance> > classSize;
    for_each(sizes.begin(), sizes.end(), boost::bind<void>(boost::ref(classSize), _1));
    stdDevSize = sqrt(variance(classSize));

    path += "/size.txt";
    ifstream size(path.c_str());

    size >> tmpDimensions[0];
    size >> tmpDimensions[1];
    size >> tmpDimensions[2];
    size >> tmpDimensions[3];
    size >> tmpDimensions[4];
    size >> tmpDimensions[5];

    return instances;

}