/***********************************************************
  Robot class for particle filter project

  ECE 2036, Spring 2036
***********************************************************/

#include "robotfield.h"

class Robot
{
public:
    explicit Robot();
    void  initializeField ( float x, float y, float orient, RobotField newfield );
    void  setPosition ( float x, float y );
    void  setOrientation ( float orient );
    void  move ( float dist, float steer, float motionError, float steeringError );
    float evaluateSensorReadingLikelihood ( float sensorReadings[], int numSensors, float sensorSTDev );
    void  getSensorReadings ( float sensorReadings[], int numSensors, float sensorError );
    [[nodiscard]] float getXPosition() const;
    [[nodiscard]] float getYPosition() const;
    [[nodiscard]] float getOrientation() const;

private:
    float xPosition;
    float yPosition;
    float orientation; // orientation in radians (between 0 and 2pi)
    RobotField field; // object created under class RobotField, which is a type of object
    int	initialized; // should be set to false in constructor and set
    // to true when the initializeField function has
    // been called so we can make sure we don't try
    // to use the RobotField before it has been
    // initialized.
}; // end class Robot
