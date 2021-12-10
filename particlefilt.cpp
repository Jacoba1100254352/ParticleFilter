/***********************************************************
    Particle filter tracking for a robot

    ECE 2036, Spring 2036
***********************************************************/

#include <iostream>		// provide cout, etc.
#include <limits>       // has the RAND_MAX value
#include <ctime>       // include time functions for random number initialization
#include <cstdlib>  	// contains function prototype for rand()
#include <random>		// A more powerful random number generator class for Gaussian random numbers

#include "robot.h"		// Our robot class, used for both the "actual" robot and the particles

// You can change the following values to explore different operating conditions
#define N_PARTICLES 	1000 // number of particles // 67500 is fun to play with
#define ITERATIONS  	10 // amount of screens shown
#define FIELDX      	100.0
#define FIELDY      	100.0
#define PRINT       	true // we want to print this

#define SENSOR_STDEV    8.0
#define MOTION_STDEV    1.0
#define STEERING_STDEV  0.1

using namespace std;	// provide easy access to cout, cin, etc.


// two non-class utility functions to be defined later in this file
void printPositions(Robot actualRobot, Robot robotParticle[], RobotField robotField, int N);
// the function prototype printPosition, will have the functions in the parentethese,
// -- pass their information on to be stored in printPosition
void resample(Robot robotParticle[], Robot robotParticleTemp[], int N, const float w[]);
// the function prototype resample, will have the functions in the parentethese,
// -- pass their information on to be stored in resample

int main()
{
    // Define variables for utility use
    int	k, its;         // loop counters
    float	newX;           // temporary variables for initializing particles
    float	newY;           // temporary variables for initializing particles
    float	newO;           // temporary variables for initializing particles
    float xavg;           // for evaluating performance at end
    float yavg;           // for evaluating performance at end
    float acx_predx_dif;  // for evaluating the difference between the actual x and predicted x
    float acy_predy_dif;  // for evaluating the difference between the actual y and predicted y

    // Define random number generators for Gaussian-distributed random numbers.
    // The following are defined in <random>.
    // Don't need to understand these, syntax is complex
    default_random_engine     	    generator;			    // set up the base random number generator object
    normal_distribution<double>	    motionNoise(0.0, MOTION_STDEV); // an object to return Gaussian random numbers based on generator
    //     parameters: mean, standard_deviation
    normal_distribution<double>     sensorNoise(0.0, SENSOR_STDEV);   // the sensor reading noise
    normal_distribution<double>     steeringNoise(0.0, STEERING_STDEV); // the steering noise

    // Define the sensor positions to be within 10 units of each corner (in the x, y directions)
    // -- as stated in the Q&A, the code is hardwired to have 4 sensors/beacons, as shone in the sensorArrayX[4],
    // -- sensorArrayY[4], and readings[4]
    // these 2 objects/functions define/show where the sensors/beacons on the Field are located, using the x and y coefficents.
    float	sensorArrayX[4] = {10.0, 10.0, FIELDX-10.0, FIELDX-10.0};   // x coefficients
    float	sensorArrayY[4]	= {10.0, FIELDY-10.0, 10.0, FIELDY-10.0};   // y coefficients

    float	readings[4];                                                // this will hold the sensor readings (distances)

    // setup up a simple object to hold the field information --
    // the field is where the robot is
    // creates object robotField for/in class RobotField
    RobotField  robotField((float) FIELDX, (float) FIELDY, sensorArrayX, sensorArrayY);

    // For a particle filter in real-life, the actual robot would be a
    // real device with sensors that returned information; we don't have
    // such a robot, so the actualRobot is an object that simulates the
    // real robot.
    Robot	actualRobot; // actualRobot is an object created in/for class Robot

    Robot	robotParticle[N_PARTICLES]; // An array of robot "particles" for our tracking.
    Robot	robotParticleTemp[N_PARTICLES]; // We have a second array for intermediate storage during resampling.
    // temp draws in the most likely bunch of particles, they move same distance but slightly random directions and
    //ultimately create more likely particles that are close to the robot for the robot to draw on.

    // Each w in the following array corresponds to a particle and
    // represents the fitness of that robot based on how well the
    // actualRobot sensor measurements match that particle's current
    // state.
    float	w[N_PARTICLES];

    ///////////////////////////////////////////////////
    // Now we need to initialize the robot particles //
    ///////////////////////////////////////////////////
// below we have robotField because // getSizex()
    srand(time(nullptr));     // initialize random seed
    newX = rand() % static_cast<int>(robotField.getSizeX());     // get random coordinates for position X and position Y
    newY = rand() % static_cast<int>(robotField.getSizeY());     // should understand rand gives you a number between 0 and RAND_MAX
    newO = static_cast<float>((rand() % 628) / 100.0);                        // and a random orientation
    /*Place the actual Robot at the new random placement in the field */
    actualRobot.initializeField(newX, newY, newO, robotField);
    // this creates an object named initializeField under actual robot that takes the data from these objects

    /* Now repeat this for each robot particle */
    // if the for loop had had k < numSensors it would go until k was greater than number of sensors,
    // in this case it will go until k is greater than the Number of Particles, and thus will not have anymore particles to look at, it terminates
    for (k = 0;  k < N_PARTICLES; k++) // k is used in different ways in almost every for loop, you set k to 0, if k is less than number of particles? run loop and add 1(++) to k
    {
        newX = rand() % static_cast<int>(robotField.getSizeX());
        newY = rand() % static_cast<int>(robotField.getSizeY());
        newO = static_cast<float>((rand() % 628) / 100.0);
        robotParticle[k].initializeField(newX, newY, newO, robotField);
// this as shown before, except rather than actual Robot, this is working with each robotParticle, thus it corresponds to k, which was explained
    }

    // Use our super-cool function to display the initial robot positions
    /* this prints the position of the actual Robot, the RobotField, the robotParticles, and the Number of particles */
    printPositions(actualRobot,robotParticle,robotField,N_PARTICLES);

    ///////////////////////////////////////////////////
    // Time for the actual PARTICLE FILTER operation //
    ///////////////////////////////////////////////////

    for (its = 0; its < ITERATIONS; its++)      // run the particle filter a few iterations defined by ITERATIONS
    {
        // move the actual robot and simulate the inexact motion and steering by including noise values
        actualRobot.move(2.0,0.2, static_cast<float>(motionNoise(generator)), static_cast<float>(steeringNoise(generator)));

        // get the robot sensor readings -- they will be returned in the array readings[] (passed by reference)
        // note: we add noise to the sensor readings also
        actualRobot.getSensorReadings(readings, 4, static_cast<float>(sensorNoise(generator)));

        // now we loop through each particle and evaluate its likelihood
        for (k = 0; k < N_PARTICLES; k++)
        {
            // w[k] represents the likelihood for particle robotParticle[k]
            w[k] = robotParticle[k].evaluateSensorReadingLikelihood(readings, 4, SENSOR_STDEV);
        }

        // here is where the magic occurs--likely particles are replicated
        resample(robotParticle, robotParticleTemp, N_PARTICLES, w);

        // Now update our display so we can visualize
        printPositions(actualRobot, robotParticle, robotField, N_PARTICLES);

        // Finally, each of our particles simulates the actual Robot by moving in the same way (with noise, of course)
        // YOU WILL NEED TO MOVE EACH OF THE PARTICLES USING THE SAME
        // PARAMETERS AS USED FOR THE ACTUALROBOT
        // ----------------------------------------
        for (k = 0; k < N_PARTICLES; k++)
        {
            robotParticle[k].move(2.0,0.2, static_cast<float>(motionNoise(generator)), static_cast<float>(steeringNoise(generator))); // there may need to be more added here
            robotParticle[k].getSensorReadings(readings, 4, static_cast<float>(sensorNoise(generator))); // I think this was all that was needed to be added
        }
        // ----------------------------------------
    }

    ///////////////////////////////////////////////////
    // Particle filtering complete--evaluate results //
    ///////////////////////////////////////////////////

    xavg = 0.0; // initialize the x average to 0.0
    yavg = 0.0; // initialize the x average to 0.0
    acx_predx_dif = 0.0; // initialize actualx_predictedx_difference to 0.0
    acy_predy_dif = 0.0; // initialize actualy_predictedy_difference to 0.0
    for (k = 0; k < N_PARTICLES; k++)
    {
        // sum up the x and y values from all particles (before final motion)
        xavg += robotParticleTemp[k].getXPosition();
        yavg += robotParticleTemp[k].getYPosition();
    }
    xavg /= N_PARTICLES;                  // calculate x average
    yavg /= N_PARTICLES;                  // calculate y average
    acx_predx_dif = fabs(xavg - actualRobot.getXPosition()); // calculate the difference
    acy_predy_dif = fabs(yavg - actualRobot.getYPosition()); // calculate the difference

    // Print results and quit!
    cout << "actual position:    " << actualRobot.getXPosition() << ", " << actualRobot.getYPosition() << endl;
    cout << "predicted position: " << xavg << ", " << yavg << endl;
    cout << "x difference between actual and predicted: " << acx_predx_dif << "\ny difference between actual and predicted: " << acy_predy_dif << endl;
// these display information at the end of the program about the accuracy of the positions
    return 0;
};



///////////////////////////////////////////////////
// Define the resampling function                //
///////////////////////////////////////////////////

void resample(Robot robotParticle[], Robot robotParticleTemp[], int N, const float w[])
{
    float       maxW = 0.0; // initializes maxW to 0
    float       step; // names function step as a float, will be used to determine where the robotparticles land
    int         index; // index object for array
    int         k; // k is our faithful for loop freind
    index = rand() % N; // index equals the remainder of rand divided by N/number of sensors
    for (k = 0; k < N; k++)
    {
        if(w[k] > maxW) //compare biggest value(maxW) with current element(w[])
        {
            // calculate maxW
            maxW = w[k];
        }
    }
    for (k = 0; k < N; k++)
    {
        step = static_cast<float>((rand() / RAND_MAX) * (2.0 * maxW));
        while(step > w[index])
        {
            step -= w[index];
            index = (index + 1) % N;
        }
        // store the selected particle
        robotParticleTemp[k] = robotParticle[index];
    }

    for (k = 0; k < N; k++) // in each sensor robotParticle[k] should equal the final, most accurate robotParticleTemp[k]
    {
        robotParticle[k] = robotParticleTemp[k];
    }
    // find the maximum weight in w[]
    // set the starting index to a random number between 0 and N-1
    // loop N times over k to randomly select particles
    // choose a random step between 0.0 and 2.0*max(w[])
// choose particles according to their measurement likelihood

    /* think back to the diagram with a circle and the slices, the w is the weight,
          the size of the slice, we want the slice to be not too big,
          so as to not have our robot take a big leap and have to figure out all over again where we are,
          and not too small, so as to not make a big enough difference when looking at particles to determine position:
          approach we are using to resampling has us conceptually arranging all the weights in a line (or circle).
          We then take a random step forward and see where we land--that is the next particle chosen
          (when we reach the end, we wrap back around to the beginning).  If our random steps are too small,
          we will just pick each particle in order, regardless of its weight.  If our random steps are too large,
          then it takes a lot of time/work to figure out where we landed.
          Choosing the random step size to be between 0 and 2*max(w) is a nice trade-off.*/
}




#define COLS 76
#define ROWS 22

void printPositions(Robot actualRobot,Robot robotParticle[], RobotField robotField, int N) // sets up the objects used in printing our current position
{
    // the next 6 objects, will be used to accomplish a certain goal
    int r; // introduces object r, that will be used to store a whole number value // these are all loop values, only to be used in a loop
    int c; // introduces object c, that will be used to store a whole number value // these are all loop values, only to be used in a loop
    int x; // introduces object x, that will be used to store a whole number value // these are all loop values, only to be used in a loop
    int y; // introduces object y, that will be used to store a whole number value // these are all loop values, only to be used in a loop
    // scale, in the next 2 objects means that the objects x, and y, will be used to ...
    float xscale; // introduces object xscale, that will be used to store a number with a value of 0.something
    float yscale; // introduces object yscale, that will be used to store a number with a value of 0.something
// the next 2 objects, will be used to accomplish a certain goal
    int putdot; // introduces object putdot, that will be used to store a whole number value
    int k; // introduces object y, that will be used to store a whole number value
// the next 2 objects, will be used to accomplish a certain goal
// max, in the next 2 objects means that the objects R, and C, will be used to ...
    float maxR; // introduces object maxR, that will be used to store a number with a value of 0.something
    float maxC; // introduces object maxC, that will be used to store a number with a value of 0.something

    if (!PRINT) return;

    maxR = robotField.getSizeY(); // maxR uses object sizeY used in robotField
    maxC = robotField.getSizeX(); // maxR uses object sizeX used in robotField

    xscale = COLS / maxC; // defines xscale as COLS divided by maxC
    yscale = ROWS / maxR; // defines yscale as ROWS divided by maxR

    cout << "+";
    for (c = 0; c < COLS; c++)
        cout << "-";
    cout << "+" << endl;
    for (r = 0; r < ROWS; r++)
    {
        cout << "|";
        for (c = 0; c < COLS; c++)
        {
            x = static_cast<int> (actualRobot.getXPosition() * xscale);
            y = static_cast<int> (actualRobot.getYPosition() * yscale);
            if ((x == c) && (y == r))
            {
                cout << "O";
            }
            else
            {
                putdot = 0;
                for (k = 0; k < N; k++)
                {
                    x = static_cast<int> (robotParticle[k].getXPosition() * xscale);
                    y = static_cast<int> (robotParticle[k].getYPosition() * yscale);
                    if ((x == c) && (y == r))
                    {
                        putdot++;
                    }
                }
                switch (putdot)
                {
                case 0 :
                    cout << " ";
                    break;
                case 1 :
                    cout << ".";
                    break;
                case 2 :
                    cout << ":";
                    break;
                case 3 :
                    cout << "=";
                    break;
                case 4 :
                    cout << "%";
                    break;
                default :
                    cout << "#";
                    break;
                }
            }
        }
        cout << "|" << endl;
    }
    cout << "+";
    for (c = 0; c < COLS; c++)
        cout << "-";
    cout << "+" << endl << endl << endl;
}
