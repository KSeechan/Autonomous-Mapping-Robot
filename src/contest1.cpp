//standard libraries
#include <ros/console.h>
#include "ros/ros.h"
#include <stdio.h>
#include <cmath>
#include <chrono>

//for collecting sensor data
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h> 
#include <tf/transform_datatypes.h> 

//to convert radians and degrees
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)


//for odometry
float posX = 0.0, posY = 0.0, yaw = 0.0;

//for bumpers and bumper events
const uint8_t N_BUMPER = 3;
uint8_t bumper[N_BUMPER] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
bool anyBumperPressed = false;
uint8_t lastBumperPressed = 0;
const float reverseLinDist = 0.1; //amount to reverse when bumper hit
const float reverseAngDist = M_PI/4; //amount to turn when bumper hit

//for lasers
float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers=0, desiredNLasers=0, desiredAngle=5;
float avgLaserDist = 0;

//robot states
const int8_t SEEK = 0;
const int8_t FREE_ROAM = 1;
const int8_t BUMPER_HIT = 2;
int8_t state;

//for movement
float angular_vel = 0.0, linear_vel = 0.0;
const float rot_slow = (M_PI/12);
const float rot_fast = (M_PI/4);
const float drive_slow = 0.1;
const float drive_fast = 0.25;

//to stay away from walls
const float safeWallDist = 0.1; //used to ensure we slow down near walls
const float startSlowingDownDist = 0.2; //set at 0.3 to give the robot enough time to slow down
const float maxDrive = 1.2; //adjust this to increase/decrease the max step the robot takes

//for drving functions
float preX = 0.0, preY = 0.0, preYaw = 0.0;
float remDist = 0.0, remRot = 0.0, currentLinear = 0.0;
float currentAngular = 0.0, distToTravel = 0.0, amtToRotate = 0.0;
bool drivingComplete = false; 
bool repeat = false;
bool headingChange = false;
float turnClockwise = 0.0, turnAntiClockwise = 0.0;

//for looking around
const uint8_t numScanSteps = 10;
const float amtRotPerStep = (2*M_PI) / numScanSteps;
float bias = 2.0; //adjust this value to bais how much the robot likes larger distances

//for roaming around
const u_int8_t lookAround = 30; //increase or decrease this value (0-100) to adjust the %chance the robot does a random scan while roamin
const u_int8_t minSpin = 45;  //should be enough to turn away from obstacles
const u_int8_t maxSpin = 140; //can be increased, 140 found to be a good value after testing

//Data Collection
//this laser function returns the minimum laser distance
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    minLaserDist = std::numeric_limits<float>::infinity();
    avgLaserDist = 0;

    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
    //ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);

    //calculate beginning and end of for loop based on desired angle
    int32_t beginIndex = 0, endIndex = nLasers;
    //ROS_INFO("endIndex: %i", endIndex);

    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        beginIndex = nLasers / 2 - desiredNLasers;
        endIndex = nLasers / 2 + desiredNLasers;
    }

    float min_dist = std::numeric_limits<float>::infinity();
    float sum_dist = 0;
    int count = 0;
    //loop through and find minimum and non zero values to get an average
    for(int i = beginIndex; i < endIndex; i++){
        float reading = msg->ranges[i];
        // Check if the reading is within a certain range
        if (reading >= msg->range_min && reading <= msg->range_max) {
            min_dist = std::min(min_dist, reading);
            sum_dist += reading;
            count++;
            //ROS_INFO("MIN_DIST Value: %.2fm", min_dist);
        }
    }
    //avg distance was calculated for debugging purposes, is not actually used.
    avgLaserDist = count > 0 ? sum_dist / count: 0;

    //set minLaserDist based on min_dist
    minLaserDist = min_dist;
    //if(minLaserDist == std::numeric_limits<float>::infinity()) minLaserDist = 0;

    //ROS_INFO("Min Laser Distance: %.2fm, Avg Laser Distance: %.2fm", minLaserDist, avgLaserDist);
}

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}

//this function tracks the turtlebots movement and assigns new drive instructions
bool trackTurtleBot(double dist, double rot, double linVel, double rotVel){
    //variables to keep track of previous values
    static double preDist = 0, preLinVel = 0, preRot = 0, preRotVel = 0;
    repeat = false;

    //check that if the call is a repeat
    if((dist == preDist) && (linVel == preLinVel) && (rot == preRot) && (preRotVel == rotVel)){
        repeat = true;
    }

    //update the previous values
    preDist = dist;
    preLinVel = linVel;
    preRot = rot;
    preRotVel = rotVel;

    drivingComplete = false;

    //if it is a repeat call, monitor the driving
    if(repeat == true){
        angular_vel = currentAngular;
        linear_vel = currentLinear;
        distToTravel = sqrt(pow(posX - preX, 2) + pow(posY - preY, 2));
        preX = posX;
        preY = posY;

        remDist = remDist - distToTravel;
        if(remDist <= 0.0) linear_vel = 0;

        amtToRotate = std::abs(yaw - preYaw);
        if (amtToRotate > M_PI){
            //ensures the amount to rotate is within pi and -pi
            amtToRotate = (M_PI - abs(yaw)) + (M_PI - abs(preYaw));
        }

        remRot = remRot - amtToRotate;
        if(remRot <= 0.0) angular_vel = 0;
        preYaw = yaw;

        currentAngular = angular_vel;
        currentLinear = linear_vel;

        if((angular_vel == 0.0) && (linear_vel == 0.0)) drivingComplete = true;

        if(drivingComplete){
            ROS_INFO("Driving Finished");
        }
    }
    //if it is a new call, assign new drive instructions
    else{
        //the remaining dist and rotation becomes the values that the function is called with
        remDist = fabs(dist);
        remRot = fabs(rot);

        preX = posX;
        preY = posY;
        preYaw = yaw;

        linear_vel = linVel;
        angular_vel = rotVel;

        currentLinear = linear_vel;
        currentAngular = angular_vel;

        ROS_INFO("New Driving Instructions - Dist:%.2f, LinVel: %.2f, rot: %.0f, rotVel: %.0f", remDist, linear_vel, remRot, angular_vel);
    }
    return drivingComplete;

}

//function to point the robot in a new direction 
bool newHeading(float targetDirection, float rotationVelocity) {
    static float previousDirection = 0;
    static float previousVelocity = 0;
    static float amountToTurn = 0;
    static float turnSpeed = 0;
    bool headingChange = false;

    if (previousDirection == targetDirection && previousVelocity == rotationVelocity) {
        // If the target direction and rotation velocity haven't changed, just call trackTurtleBot with the same parameters
        headingChange = trackTurtleBot(0, amountToTurn, 0, turnSpeed);
    } else {
        // If the target direction or rotation velocity have changed, update the stored values
        previousDirection = targetDirection;
        previousVelocity = rotationVelocity;

        // Calculate the amount to turn and the direction to turn
        float deltaDirection = targetDirection - yaw;
        //ensures that the change is always within pi and -pi
        while (deltaDirection > M_PI) deltaDirection -= 2*M_PI;
        while (deltaDirection < -M_PI) deltaDirection += 2*M_PI;

        bool turnClockwise = deltaDirection < 0;
        amountToTurn = fabs(deltaDirection);

        // Set the turn speed based on the direction to turn
        turnSpeed = (turnClockwise ? -1 : 1) * rotationVelocity;

        // Call trackTurtleBot with the calculated amount to turn and rotation velocity
        headingChange = trackTurtleBot(0, amountToTurn, 0, turnSpeed);
    }

    return headingChange;
}


//Obstacle Interaction Functions
//function to check bumpers
void bumperEventCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    bumper[msg->bumper] = msg->state;

    if (msg->state == kobuki_msgs::BumperEvent::RELEASED){
        ROS_INFO("%d released", msg->bumper);
    } else if (msg->state == kobuki_msgs::BumperEvent::PRESSED) {
        anyBumperPressed = true;
        ROS_INFO("%d bumper impact detected.", msg->bumper);
        state = BUMPER_HIT;
        lastBumperPressed = msg->bumper;
    }
}

//function to react to a bumper being pressed
void bumperHitMovement(u_int8_t &subState) {
    //check the subState pass in from the main function
    switch(subState) {
    //when substate = 0, reverse away from the obstacle and increment the substate by 1
        case 0:
            if(trackTurtleBot(0, 0, 0, 0)){
                subState++;
                ROS_INFO("Bumper Hit detected, stoping robot");
            }
            break;
    //when substate = 1, rotate the robot depending on which bumper got pressed.
        case 1:
            if(trackTurtleBot(reverseLinDist, 0, (-drive_fast), 0)){
                ROS_INFO("Done reversing from hit.");
                subState++;
            }
            break;
        case 2:  
            switch(lastBumperPressed){
                case 0:
                    //rotate 45 degrees to right when left bumper pressed.
                    if(trackTurtleBot(0, reverseAngDist, 0, (-rot_fast))){
                        ROS_INFO("Done rotating from left hit bumper.");
                        state = FREE_ROAM;
                    }
                    break;

                case 1:
                    //rotate 90 degrees to right when bumper pressed
                    if(trackTurtleBot(0, (reverseAngDist*2), 0, rot_fast)){
                        ROS_INFO("Done rotating from middle hit bumper.");
                        state = FREE_ROAM;
                    }
                    break;
                
                case 2:
                    //rotate 45 degrees to the left when right bumper pressed
                    if(trackTurtleBot(0, reverseAngDist, 0, rot_fast)){
                        ROS_INFO("Done rotating from right hit bumper.");
                        state = FREE_ROAM;
                    }
                    break;
            }
            break;
    }
}


//LOOK AROUND FUNCTIONS 
//function to look around in a circle, turns the robot 360 degrees in 10 steps
//once the scan is complete, this function determines the direction to point the robot in
float findIdealDirection(const float yawPos[], const float minDist[], float power){
    //to find the ideal direction, randomly select a heading based on the probabilities proportional to the distances.
    float sumDists = 0;
    int idealIndex = 0;
    float scores[numScanSteps];
    float randomValue = 0;
    float sumScores = 0;

    //calculate the sum of all the distances returned from minDist[], applies a power factor to increase the chance of choosing a large distance
    for(int i = 0; i < numScanSteps; i++){
        sumDists += pow(minDist[i], power);
    }

    //if all the distances are 0, sumDists=0 and there is an error
    if (sumDists == 0){
        ROS_INFO("Scan unsuccessful");
        return yaw;
    }

    //give each scan step a score
    for(int i = 0; i < numScanSteps; i++){
        scores[i] = pow(minDist[i], power) / sumDists;
    }

    //select a random step based on the scores
    randomValue = sumDists * (rand() / (float)RAND_MAX);
    for(int i = 0; i < numScanSteps; i++){
        sumScores += scores[i];
        if (sumScores >= randomValue){
            idealIndex = i;
            break;
        }
    }

    return yawPos[idealIndex];
}


void circularScan(uint8_t &subState) {
    static float yawPositions[numScanSteps];
    static float minDistances[numScanSteps];
    static float idealDirection;
    static uint8_t scanSteps = 0;
    static bool scanCompleted = false;

    switch(subState) {
        case 0:
            // Initialize scan
            scanSteps = 0;
            subState = 1;
            ROS_INFO("Beginning 360 Scan of Environment");
            break;

        case 1:
            // Rotate the robot and store the distances and yaw obtained for each scan step
            scanCompleted = trackTurtleBot(0, amtRotPerStep, 0, rot_fast);

            if (scanCompleted) {
                // Data collection completed for this scan step
                yawPositions[scanSteps] = yaw;
                minDistances[scanSteps] = minLaserDist;
                scanSteps++;
                ROS_INFO("Step#:%d, Distance:%.2f, Yaw:%.0f", scanSteps, minLaserDist, yaw);

                // Stop the robot at each step
                if (scanSteps < 10) {
                    trackTurtleBot(0, 0, 0, 0);
                }
                else {
                    // All data has been collected, find the ideal direction to drive
                    idealDirection = findIdealDirection(yawPositions, minDistances, bias);
                    subState = 2;
                    ROS_INFO("After scan, ideal direction found to be %.0f", RAD2DEG(idealDirection));
                }
            }
            break;

        case 2:
            // Point the robot in the ideal direction and set to free roam
            scanCompleted = newHeading(idealDirection, rot_fast);

            if (scanCompleted) {
                state = FREE_ROAM;
            }
            break;

        default:
            // Invalid substate, do nothing
            break;
    }
}




//Functions for Roaming Around
//this function tries minimize robot turning after each drive step to bais the robot to travel forward
float driveAngle(float minSpinVal, float maxSpinVal){
    float range = maxSpinVal - minSpinVal;
    float alpha = 5.0; // Choose a value for alpha (larger than beta to skew angle to minSpinVal)
    float beta = 2.0; // Choose a value for beta
    float u = static_cast<float>(rand()) / RAND_MAX;
    //calculates the angle based on the beta probability distribution
    float angle = minSpinVal + range * pow(u, alpha) * pow(1 - u, beta);
    return angle;
}


int8_t roaming(u_int8_t &subState) {
    static float driveStep = 0, driveVel = 0;
    static bool firstDrive = true;

    switch (subState) {
        //when subState is 0, calculate the drive step and velocity 
        case 0: {
            driveStep = std::max(0.0f, minLaserDist - safeWallDist);
            driveStep = std::min(maxDrive, driveStep);
            //ensure that the slow speed is set if we are close to obstacles/walls
            driveVel = ((minLaserDist - driveStep) < startSlowingDownDist) ? drive_slow : drive_fast;
            subState++;
            break;
        }

        case 1: {
            ROS_INFO("Free Roaming forward %.2f m at a velocity of %.2f m/s", driveStep, driveVel);
            bool finishedDriving = trackTurtleBot(driveStep, 0, driveVel, 0);
            if (finishedDriving) {
                //calculate a random value
                u_int8_t diceRoll = rand() % 100;
                ROS_INFO("diceRoll: %i", diceRoll);
                //if this random value is less than lookAround, take a new 360 scan
                if (diceRoll < lookAround) {
                    state = SEEK;
                    return state;
                }
                else {
                    //calculate the new drive step and velocity if the robot decides to take another step
                    if (minLaserDist < (startSlowingDownDist)) {
                        //if there is an obstacle ahead, calculate a new drive step based on minSpin and maxSpin
                        driveStep = DEG2RAD(driveAngle(minSpin, maxSpin));
                    }
                    else {
                        //if no obstacle present, allow no spin in the calculation
                        driveStep = DEG2RAD(driveAngle(0, maxSpin));
                    }
                    //random choice of drive velocity between fast clockwise or anti clockwise
                    driveVel = ((rand() % 2) == 1) ? rot_fast : (-rot_fast);
                    if (firstDrive) {
                        firstDrive = false;
                        subState = 2;
                    }
                    else {
                        subState = 3;
                    }
                }
            }
            break;
        }

        case 2: {
            //send the new driving instructions
            bool finishedDriving = trackTurtleBot(driveStep, 0, driveVel, 0);
            if (finishedDriving) {
                firstDrive = true;
                subState++;
            }
            break;
        }

        case 3: {
            //rotates the robot if an obstacle is in front
            if (minLaserDist < (startSlowingDownDist)) {
                driveStep = 1.5 * driveStep;
                driveVel = -driveVel;
                subState = 4;
            }
            else {
                subState = 0;
            }
            break;
        }

        case 4: {
            //sends the rotate instructions
            bool finishedDriving = trackTurtleBot(0, driveStep, 0, driveVel);
            if (finishedDriving) {
                subState = 0;
            }
            break;
        }
    }
}



int main(int argc, char **argv)
{
    //initilise ROS
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    //subscribe to bumper, laser and odometry topics
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperEventCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);
    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    //variables to keep track of states and subStates
    u_int8_t preState = 0;
    u_int8_t subState = 0;
    u_int8_t loopCount = 0;

    //get one set of calls before starting to loop
    ros::spinOnce();
    loop_rate.sleep();
    
    //set the state to SEEK so that the robot scans its environment first
    state = SEEK;

    //start the loop
    ROS_INFO("STARTING THE LOOP");
    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        loopCount++;
        
        //check what the state is
        if(preState != state){
            preState = state;
            subState = 0;
            loopCount = 0;
        }

        //tell the robot to act based on the current state
        switch (state) {
            case BUMPER_HIT:
                bumperHitMovement(subState);
            break;
            case FREE_ROAM:
                roaming(subState);
            break;
            case SEEK:
                circularScan(subState);
             break;
        }


        //publish velocity command
        vel.angular.z = angular_vel;
        vel.linear.x = linear_vel;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
