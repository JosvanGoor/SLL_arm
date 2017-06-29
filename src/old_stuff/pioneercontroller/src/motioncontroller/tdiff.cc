#include <pioneercontroller2/motioncontroller.h>
#include <ctime>

float MotionController::tdif(timespec t1, timespec t2)
{
    /* Returns the positive difference between t2 and t2, in seconds, with nanosecond precision */
    float secdif = (t2.tv_sec - t1.tv_sec);
    float nsecdif = (t2.tv_nsec - t1.tv_nsec);
    float totaldif = secdif + nsecdif/1e9;
    float absdif = totaldif > 0 ? totaldif : -totaldif;
    return absdif;
}

