#include <pioneercontroller2/motioncontroller.h>

// turn on motors, and off sonar, and off amigobot sounds, when connected
void MotionController::on_connected(void)
{
    printf("[MotionController] Robot connection handler: Connected\n");
    d_lock.lock();
    d_error = NO_ERROR;
    if (d_tmp_robot != 0)
    {
        d_tmp_robot->comInt(ArCommands::SONAR, 0);
        d_tmp_robot->comInt(ArCommands::ENABLE, 1);
        d_tmp_robot->comInt(ArCommands::SOUNDTOG, 0);
    }
    if (d_robot != 0)
    {
        d_tmp_robot->comInt(ArCommands::SONAR, 0);
        d_tmp_robot->comInt(ArCommands::ENABLE, 1);
        d_tmp_robot->comInt(ArCommands::SOUNDTOG, 0);
    }
    d_lock.unlock();
}
