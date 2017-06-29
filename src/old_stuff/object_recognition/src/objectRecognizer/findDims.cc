#include "objectRecognizer.ih"

void ObjectRecognizer::findDims(vector<double> xs, vector<double> ys, vector<double> zs, vector<double> &tmpDimensions)
{
    double stdDevX;
    double stdDevY;
    double stdDevZ;

    accumulator_set<double, stats<tag::variance> > accX;
    for_each(xs.begin(), xs.end(), boost::bind<void>(boost::ref(accX), _1));
    stdDevX = sqrt(variance(accX));

    accumulator_set<double, stats<tag::variance> > accY;
    for_each(ys.begin(), ys.end(), boost::bind<void>(boost::ref(accY), _1));
    stdDevY = sqrt(variance(accY));

    accumulator_set<double, stats<tag::variance> > accZ;
    for_each(zs.begin(), zs.end(), boost::bind<void>(boost::ref(accZ), _1));
    stdDevZ = sqrt(variance(accZ));

    double min_x = *min_element(xs.begin(),xs.end());
    double max_x = *max_element(xs.begin(),xs.end());
    double min_y = *min_element(ys.begin(),ys.end());
    double max_y = *max_element(ys.begin(),ys.end());

    tmpDimensions[0] =  min_x - (2 * stdDevX);
    tmpDimensions[1] =  max_x + (2 * stdDevX);
    tmpDimensions[2] =  min_y - (2 * stdDevY);
    tmpDimensions[3] =  max_y + (2 * stdDevY);
    tmpDimensions[4] = min_x / max_y;
    tmpDimensions[5] = max_x / min_y;

}