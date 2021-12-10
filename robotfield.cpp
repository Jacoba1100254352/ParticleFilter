#include "robotfield.h"
/* Robot Field keeps track of the position and dimentions of the Field, will be the same for each robot,
helps track where the "robots" particles, each robot and particle will use this,
when 1 robot uses it, it will not be changed for the other robots. */
RobotField::RobotField()
{
    sizeX	= 100.0; // Size of plane X is 100
    sizeY	= 100.0; // Size of plane Y is 100
    numSensors = 4; // stating the number of sensors used, do not change, hardwierd into code
    // k is a loop variable, only for use in for loops, k is declared in main and a few other functions for temporary use
    for ( int k = 0; k < numSensors; k++ ) // k is 0, k is less than the number of sensors ad 1 to k is greater than the number of Sensors,
    {
        // thus showing that all the sensors have been checked in this loop
        sensorX[k] = 0.0;
        sensorY[k] = 0.0;
    }

}

RobotField::RobotField ( float x, float y, const float X[4], const float Y[4] )
{
    sizeX	= x; // nameing sizeX as x, important for use when looking at later code!
    sizeY	= y; // nameing sizeY as y, important for use when looking at later code!
    numSensors = 4; // stating the number of sensors used, do not change, hardwierd into code

    for ( int k = 0; k < numSensors; k++ ) // evaluate this in light of previous code.
    {
        sensorX[k] = X[k]; // this may be nameing object/function sensorX[k] as X[k]
        sensorY[k] = Y[k]; // this may be nameing object/function sensorY[k] as Y[k]
    }// at first X[k] (sub) k=0, run the loop, X[0], then X[1], then X[2], and last X[3]
    // runs through the first Sensor in the X position, naming it X[k] for short, keeps looping until it has seen all sensors for X and Y
    // runs through the first Sensor in the Y position, naming it Y[k] for short, keeps looping until it has seen all sensors for X and Y
    // then all the sensors have been rotated through, k is now greater than the number of sensors so the for loop does not run again

}
float RobotField::getSizeX() const
{
    return sizeX;
}
float RobotField::getSizeY() const
{
    return sizeY;
}
float RobotField::getSensorX ( int num )
{
    return sensorX[num];
}
float RobotField::getSensorY ( int num )
{
    return sensorY[num];
}
