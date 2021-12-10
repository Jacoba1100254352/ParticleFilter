/***********************************************************
    Robot class for particle filter project

    ECE 2036, Spring 2036
***********************************************************/

#include <iostream>
#include <cmath>		// necessary for sin() and cos()
#include "robot.h"
using namespace std;

#define PI2 6.283185307179586
// Notes on the Robot class:
// The robot class should flag an error if methods are called before
// initialization is complete.

// Note, if the position is set outside of the field, put it at the edge

// Keep the orientation between 0 and 2pi

// The Move function move should update the orientation by adding the
// steering and steering noise to the current orientation.
// move should then add motion error to the distance and move that
// distance in the direction determined by orientation (you can use
// cos() and sin() ).
Robot::Robot()
{
    xPosition = 0;
    yPosition = 0;
    orientation = 0;
    initialized = true; // if set to 0 or false it will not allow the program to terminate as far as I can see
}

float Robot::evaluateSensorReadingLikelihood ( float sensorReadings[], int numSensors, float sensorSTDev )
{
    float dx = 0.0;
    float dy = 0.0;
    int N = numSensors;
    float disterror = 0.0; // look back over, may not need the 0
    float likelihood = 1.0;
    float distance = 0.0; // distance stands for the Euclidean distance which is the square root of (x1 - xb)^2 + (y1 - yb)^2
    float noisevar = sensorSTDev * sensorSTDev; // sensorSTDev tells you how spread out the noise can be
    if ( !initialized )
    {
        cout << "initialize robot before reading sensors" << endl;
        return 0;
    }

    // ensures that the field is initialized, used in both functions // will
    // evaluate the Gaussian function for each distance error to get its likelihood; the total likelihood is the product of all likelihoods
    for ( int k = 0; k < N; k++ )
    {
        // find the distance error (difference) between our sensor distance and the distance readings that are evaluating for each sensor beacon
        dx = getXPosition() - field.getSensorX ( k ); // check the number of sensors vs. our field object
        dy = getYPosition() - field.getSensorY ( k ); // find the distance from our position to each sensor (don't add noise here)
        distance = sqrt ( dx * dx + dy * dy ); // Euclidean distance
        disterror = fabs ( distance - sensorReadings[k] );
        // figure out the distance error by taking the distance and subtracting it by the readings and making sure it doesn't go negative
        likelihood *= static_cast<float>(exp ( -disterror * disterror / ( 2.0 * noisevar ) ) / sqrt ( ( PI2 ) * noisevar ));
    }

    /* likelihood times-equals the exponent of (distance-error times distance-error divided by
    (2 times noisevariance [which is sensorSTDev times sensorSTDev]))
    divided by the square root of ((pi times 2) times noisevariance [which is sensorSTDev times sensorSTDev]) */

    return likelihood;

}

// *sensorReadings and sensorReadings[], are the same and show that they are a pointer
// in Robot class, getSensorReadings will get the readings from the 3 listed variables/objects
void Robot::getSensorReadings ( float sensorReadings[], int numSensors, float sensorError )
{
    float	dx = 0.0;			// x distance to sensor
    float	dy = 0.0;			// y distance to sensor
    int N = numSensors; // as seen below, N may need to be changed to number of particles, depends on what's used in the for loop
    if ( !initialized ) // if !(not) initialized...
    {
        cout << "initialize robot before reading sensors" << endl;
        return; // needed to make sure it is initialized, will warn you if it is not
    }
    for ( int k = 0; k < N; k++ )
    {
        dx = getXPosition() - field.getSensorX ( k );
        dy = getYPosition() - field.getSensorY ( k ); // N = number of particls or number of sensors, whatever I use it for in this loop, need to define before hand eg N = numsensors etc.
        sensorReadings[k] = sqrt ( dx * dx + dy * dy ) + sensorError;
    }
    /* for the resample, takes a jump over a series of slots, subtracts each slot until it gets below or on 0,
    so that it knows where, on the slot it is, if you make it too big, lots of slot subtrating, too small,
    and it doesn't go anywhere. (actual code has been supplied for this, not psuedo-code
      while (maximum > v[k]); */
    /* return a pointer in the array, accuracy needs to be within about 12-10 units,
    if your not sure if something is wrong bump up the particles */
}
// describe the objects, variables in initializeField
void 	Robot::initializeField ( float x, float y, float orient, RobotField newfield )
{
    setPosition ( x, y ); // groups xposition and yPosition into this object, described below
    setOrientation ( orient ); // sets Orientation as orient, described below
    field = newfield; //  field from robot.h = variable newfield
}
// describe the objects, variables in initializeField
void 	Robot::setPosition ( float x, float y )
{
    xPosition = x; // xPosition put into variable x for ease of use
    yPosition = y; // yPosition put into variable y for ease of use
}
// describe the objects, variables in Orientation
void  Robot::setOrientation ( float orient )
{
    orientation = orient; // orientation put into variable orient
}
void 	Robot::move ( float dist, float steer, float motionError, float steeringError )
{
    float orient;
    float distnoise;
    float deltaX;
    float deltaY;
    // describes orient
    orient = getOrientation();
    orient += steer + steeringError;
    setOrientation ( orient );
    // describes distance noise
    distnoise = dist + motionError;
    deltaX = distnoise * cos ( getOrientation() );
    deltaY = distnoise * sin ( getOrientation() );

    setPosition ( getXPosition() + deltaX, getYPosition() + deltaY ); // explain setPosition for below use
    if ( getXPosition() > field.getSizeX() ) // makes sure that the actualrobot does not get overprinted
    {
        setPosition ( static_cast<float>(field.getSizeX() - 0.01), getYPosition() );
    }
    if ( getYPosition() > field.getSizeY() ) // makes sure that the actualrobot does not get overprinted
    {
        setPosition ( getXPosition(), static_cast<float>(field.getSizeY() - 0.01) );
    }
    if ( getXPosition() < 0 ) // makes sure that the actualrobot does not get overprinted
    {
        setPosition ( 0, getYPosition() );
    }
    if ( getYPosition() < 0 ) // makes sure that the actualrobot does not get overprinted
    {
        setPosition ( getXPosition(), 0 );
    }
}
// So, in response to your first question about motionError and steeringError: the robot moves with noise --
// if I tell it to move 2 units, it might move 2.1 units, or 1.9 units, or 2.0 units, or something in between.
// motionError is an error that you need to add to the distance, to simulate realistic noisy robot movement.
// steeringError is the same thing, but for the steer.
// The Move function move should update the orientation by adding the
// steering and steering noise to the current orientation.
// move should then add motion error to the distance and move that
// distance in the direction determined by orientation (you can use
// cos() and sin() ).
float Robot::getXPosition() const
{
    return xPosition; // when getXPosition called, return xPosition
}
float Robot::getYPosition() const
{
    return yPosition; // when getXPosition called, return yPosition
}
float Robot::getOrientation() const
{
    return orientation; // when getOrientation called, return orientation
}
