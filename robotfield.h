class RobotField
{
public:
    explicit RobotField();
    RobotField ( float x, float y, const float X[4], const float Y[4] );
    [[nodiscard]] float   getSizeX() const; // need to check sizeX, moved to private
    [[nodiscard]] float   getSizeY() const; // makes it necesary to call, before they can be used, fundementals cannot be changed

    int		numSensors = 4; // there are 4 sensors

    float	getSensorX ( int num ); // creates variable num
    float	getSensorY ( int num ); // use num when using getSensorY

private:
    float	sizeX; // made the public data members private
    float	sizeY; // makes it necesary to call, before they can be used, fundementals cannot be changed

    float	sensorX[4]{}; // there are 4 sensors, each with an X position
    float	sensorY[4]{}; // there are 4 sensors, each with an Y position
};
