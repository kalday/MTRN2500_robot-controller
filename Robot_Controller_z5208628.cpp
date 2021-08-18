// File:          Robot_Controller_z5208628.cpp
// Date:          Monday, 12 October 2020
// Description:   MTRN2500: Assignment 1 Task
// Author:        Kyra Elyse Alday

// Webots Headers
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Keyboard.hpp>

// Standard Libraries
#include <iostream>
#include <array>
#include <string>
#include <iomanip>
#include <vector>
#include <fstream>
#include <algorithm>
#include <numeric>

constexpr double MAX_MOTOR_SPEED{10};       // Robot Motor Max Speed (rad/s)
constexpr int DISTANCE_SENSOR_NUMBER{4};    // Number of Distance Sensors
constexpr double OBSTACLE_THRESHOLD{800.0}; // Ostacle Detection Threshold

// -------------------------------------- TASK_MANAGER Class ------------------------------------- //
class taskManager
{
public:
    taskManager(std::string managerName) : mManagerName{managerName} {};
    void printCommandList() const; // print function for command list
private:
    std::string mManagerName; // TASK_MANAGER name
};

void taskManager::printCommandList() const
{ // print function for command list
    std::cout << mManagerName << ": Please select the command:" << std::endl;
    std::cout << mManagerName << ": [1] run ROBOT_RED in manual-control mode and write data to ROBOT_RED.csv" << std::endl;
    std::cout << mManagerName << ": [2] run ROBOT_BLUE in manual-control mode and write data to ROBOT_BLUE.csv" << std::endl;
    std::cout << mManagerName << ": [3] run ROBOT_RED and ROBOT_BLUE in wall-following mode" << std::endl;
    std::cout << mManagerName << ": [4] run ROBOT_RED or ROBOT_BLUE in shortest-time mode" << std::endl;
    std::cout << mManagerName << ": [5] read data from ROBOT_RED.csv and process" << std::endl;
    std::cout << mManagerName << ": [6] read data from ROBOT_BLUE.csv and process" << std::endl;
}

// -------------------------------------- ROBOT_RED Class --------------------------------------- //
class robotRed
{
public:
    robotRed(std::string robotName) : mRobotName{robotName} {}; // constructor : member initialiser list
    void printRedCommands() const;                              // function for printing command list
    // data reading
    void robotRedSummary(std::vector<double> &virtualTime, std::vector<double> &rawReadingDS0,
                         std::vector<double> &rawReadingDS1, std::vector<double> &rawReadingDS2,
                         std::vector<double> &rawReadingDS3) const; // function for data processing
private:
    std::string mRobotName;
    std::string mFileName;
    char mDelim; // delimitor
};

void robotRed::printRedCommands() const
{ // print function for red command list
    std::cout << mRobotName << ": Starting writing data ROBOT_RED.csv" << std::endl;
    std::cout << mRobotName << ": Please use the following commands to control the motion:" << std::endl;
    std::cout << mRobotName << ": [W]     Move forward" << std::endl;
    std::cout << mRobotName << ": [S]     Move backward" << std::endl;
    std::cout << mRobotName << ": [A]     Turn left" << std::endl;
    std::cout << mRobotName << ": [D]     Turn right" << std::endl;
    std::cout << mRobotName << ": [SPACE] Stop" << std::endl;
}

void robotRed::robotRedSummary(std::vector<double> &virtualTime, std::vector<double> &rawReadingDS0,
                               std::vector<double> &rawReadingDS1, std::vector<double> &rawReadingDS2,
                               std::vector<double> &rawReadingDS3) const
{
    // elements to find the maximum numbers
    auto maxTime = *max_element(virtualTime.cbegin(), virtualTime.cend());
    auto maxDS0 = *max_element(rawReadingDS0.cbegin(), rawReadingDS0.cend());
    auto maxDS1 = *max_element(rawReadingDS1.cbegin(), rawReadingDS1.cend());
    auto maxDS2 = *max_element(rawReadingDS2.cbegin(), rawReadingDS2.cend());
    auto maxDS3 = *max_element(rawReadingDS3.cbegin(), rawReadingDS3.cend());

    // elements to find the minimum numbers
    auto minTime = *min_element(virtualTime.begin(), virtualTime.end());
    auto minDS0 = *min_element(rawReadingDS0.begin(), rawReadingDS0.end());
    auto minDS1 = *min_element(rawReadingDS1.begin(), rawReadingDS1.end());
    auto minDS2 = *min_element(rawReadingDS2.begin(), rawReadingDS2.end());
    auto minDS3 = *min_element(rawReadingDS3.begin(), rawReadingDS3.end());

    // finding the averages of the vectors
    auto avgTime = accumulate(virtualTime.begin(), virtualTime.end(), 0.0) / virtualTime.size();
    auto avgDS0 = accumulate(rawReadingDS0.begin(), rawReadingDS0.end(), 0.0) / rawReadingDS0.size();
    auto avgDS1 = accumulate(rawReadingDS1.begin(), rawReadingDS1.end(), 0.0) / rawReadingDS1.size();
    auto avgDS2 = accumulate(rawReadingDS2.begin(), rawReadingDS2.end(), 0.0) / rawReadingDS2.size();
    auto avgDS3 = accumulate(rawReadingDS3.begin(), rawReadingDS3.end(), 0.0) / rawReadingDS3.size();

    // printing out maximum, minimum and average number summary
    std::cout << mRobotName << ": The maximum values are - " << maxTime << ", " << maxDS0 << ", " << maxDS1 << ", " << maxDS2 << ", " << maxDS3 << std::endl;
    std::cout << mRobotName << ": The minimum values are - " << minTime << ", " << minDS0 << ", " << minDS1 << ", " << minDS2 << ", " << minDS3 << std::endl;
    std::cout << mRobotName << ": The average values are - " << avgTime << ", " << avgDS0 << ", " << avgDS1 << ", " << avgDS2 << ", " << avgDS3 << std::endl;
}

// -------------------------------------- ROBOT_BLUE Class --------------------------------------- //
class robotBlue
{
public:
    robotBlue(std::string robotName, std::string sortedFile, char delim = ',')
        : mRobotName{robotName}, mSortedFile{sortedFile}, mDelim{delim} {}; // constructor : member initialiser list
    void printBlueCommands() const;                                         // function for printing command list
    // data reading
    std::vector<std::vector<double>> sortData(std::vector<std::vector<double>> &data); // function for sorting out the data
    void writeSortedData(std::vector<std::vector<double>> &data) const;                // function for writing out the sorted data
private:
    std::string mRobotName;
    std::string mSortedFile;
    char mDelim; // delimitor
};

void robotBlue::printBlueCommands() const
{ // print function for blue command list
    std::cout << mRobotName << ": Starting writing data ROBOT_BLUE.csv" << std::endl;
    std::cout << mRobotName << ": Please use the following commands to control the motion:" << std::endl;
    std::cout << mRobotName << ": [UP]    Move forward" << std::endl;
    std::cout << mRobotName << ": [DOWN]  Move backward" << std::endl;
    std::cout << mRobotName << ": [LEFT]  Turn left" << std::endl;
    std::cout << mRobotName << ": [RIGHT] Turn right" << std::endl;
    std::cout << mRobotName << ": [SPACE] Stop" << std::endl;
}

std::vector<std::vector<double>> robotBlue::sortData(std::vector<std::vector<double>> &data)
{
    // sorting by the value of the 5th column
    std::sort(data.begin(), data.end(),
              [](const std::vector<double> &a, const std::vector<double> &b) {
                  return a[4] < b[4];
              });

    return data;
}

void robotBlue::writeSortedData(std::vector<std::vector<double>> &data) const
{
    std::ofstream fout{mSortedFile, std::ios::out | std::ios::app}; // open file in append mode

    if (fout.is_open()) // check if open successfully
    {
        for (auto iter = data.cbegin(); iter != data.cend() - 1; ++iter) // write elements (except the last) of dataLine to file, delimited by ','
        {
            for (auto iter1 = (*iter).cbegin(); iter1 != (*iter).cend() - 1; iter1++)
            {
                fout << std::fixed << std::setprecision(3) << *iter1 << mDelim;
            }

            fout << std::fixed << std::setprecision(3) << (*iter).back() << std::endl; // write the last element of dataLine to file, followed by '\n' and flush
        }

        // print message
        std::cout << mRobotName << ": Writing data to " + mSortedFile + " suceeded!" << std::endl;
    }
    else // error message
    {
        throw std::runtime_error(mRobotName + ": Writing new line to " + mSortedFile + " failed!");
    }
}

// ------------------------------------ ROBOT_CONTROL Class ------------------------------------- //

class robotControl
{
public:
    robotControl(std::string robotName, std::string fileName, webots::Robot &robot, char delim = ',')
        : mRobotName{robotName}, mFileName{fileName}, mRobot{robot}, mDelim{delim} {};
    // manual movement
    void robotForward();  // robot forward command
    void robotBackward(); // robot backward command
    void robotLeft();     // robot left command
    void robotRight();    // robot right command
    void robotStop();     // robot stop command
    // data loading
    void loadRobotData(std::array<double, DISTANCE_SENSOR_NUMBER> dsValues, std::vector<double> &dataLine, webots::Robot &robot) const; // function for printing command list
    void writeRobotFile(const std::vector<double> &dataLine) const;                                                                     // writing to file
    // navigation
    void robotFollowWall(std::array<webots::DistanceSensor *, DISTANCE_SENSOR_NUMBER> ds, int timeStep);                    // function for wall-following mode
    void shortestPath(std::array<webots::DistanceSensor *, DISTANCE_SENSOR_NUMBER> ds, int timeStep, webots::Robot &robot); // function for the shortest maze path
    // data reading
    std::vector<std::vector<double>> readRobotData(std::vector<double> &virtualTime, std::vector<double> &rawReadingDS0,
                                                   std::vector<double> &rawReadingDS1, std::vector<double> &rawReadingDS2,
                                                   std::vector<double> &rawReadingDS3) const; // read data from csv
private:
    std::string mRobotName;
    std::string mFileName;
    webots::Robot mRobot;
    char mDelim; // delimitor
};

void robotControl::robotForward()
{
    // get the motor devices
    webots::Motor *leftMotor = mRobot.getMotor("left wheel motor");
    webots::Motor *rightMotor = mRobot.getMotor("right wheel motor");

    // set up the motor speeds at 10% of the MAX_MOTOR_SPEED.
    leftMotor->setVelocity(0.5 * MAX_MOTOR_SPEED);
    rightMotor->setVelocity(0.5 * MAX_MOTOR_SPEED);
}

void robotControl::robotBackward()
{
    // get the motor devices
    webots::Motor *leftMotor = mRobot.getMotor("left wheel motor");
    webots::Motor *rightMotor = mRobot.getMotor("right wheel motor");

    // set up the motor speeds at 10% of the MAX_MOTOR_SPEED.
    leftMotor->setVelocity(-0.5 * MAX_MOTOR_SPEED);
    rightMotor->setVelocity(-0.5 * MAX_MOTOR_SPEED);
}

void robotControl::robotLeft()
{
    // get the motor devices
    webots::Motor *leftMotor = mRobot.getMotor("left wheel motor");
    webots::Motor *rightMotor = mRobot.getMotor("right wheel motor");

    // set up the motor speeds at 10% of the MAX_MOTOR_SPEED.
    leftMotor->setVelocity(-0.25 * MAX_MOTOR_SPEED);
    rightMotor->setVelocity(0.5 * MAX_MOTOR_SPEED);
}

void robotControl::robotRight()
{
    // get the motor devices
    webots::Motor *leftMotor = mRobot.getMotor("left wheel motor");
    webots::Motor *rightMotor = mRobot.getMotor("right wheel motor");

    // set up the motor speeds at 10% of the MAX_MOTOR_SPEED.
    leftMotor->setVelocity(0.5 * MAX_MOTOR_SPEED);
    rightMotor->setVelocity(-0.25 * MAX_MOTOR_SPEED);
}

void robotControl::robotStop()
{

    // get the motor devices
    webots::Motor *leftMotor = mRobot.getMotor("left wheel motor");
    webots::Motor *rightMotor = mRobot.getMotor("right wheel motor");

    // set up the motor speeds at 10% of the MAX_MOTOR_SPEED.
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
}

void robotControl::loadRobotData(std::array<double, DISTANCE_SENSOR_NUMBER> dsValues, std::vector<double> &dataLine, webots::Robot &robot) const
{                                        // pushing back values into the dataLine
    dataLine.push_back(robot.getTime()); // Virtual_Time
    dataLine.push_back(dsValues[0]);     // Raw_Reading_ds0
    dataLine.push_back(dsValues[1]);     // Raw_Reading_ds1
    dataLine.push_back(dsValues[2]);     // Raw_Reading_ds2
    dataLine.push_back(dsValues[3]);     // Raw_Reading_ds3
}

void robotControl::writeRobotFile(const std::vector<double> &dataLine) const

{
    std::ofstream fout{mFileName, std::ios::out | std::ios::app}; // open file in append mode

    if (fout.is_open()) // check if open successfully
    {
        for (auto iter = dataLine.begin(); iter != dataLine.end() - 1; ++iter) // write elements (except the last) of dataLine to file, delimited by ','
        {
            fout << std::fixed << std::setprecision(3) << *iter << mDelim;
        }

        fout << std::fixed << std::setprecision(3) << dataLine.back() << std::endl; // write the last element of dataLine to file, followed by '\n' and flush
    }
    else // error message
    {
        throw std::runtime_error(mRobotName + ": Writing new line to " + mFileName + " failed!");
    }
}

void robotControl::robotFollowWall(std::array<webots::DistanceSensor *, DISTANCE_SENSOR_NUMBER> ds, int timeStep)
{
    std::cout << mRobotName << ": Starting wall-following mode at " << mRobot.getTime() << " s" << std::endl; // print message

    // enable motors
    webots::Motor *leftMotor = mRobot.getMotor("left wheel motor");
    webots::Motor *rightMotor = mRobot.getMotor("right wheel motor");

    // get the camera device and enable cameras
    webots::Camera *camera = mRobot.getCamera("camera");
    camera->enable(timeStep);
    int width = camera->getWidth();
    int height = camera->getHeight();

    while (mRobot.step(timeStep) != -1)
    {
        // read sensors outputs
        std::array<double, DISTANCE_SENSOR_NUMBER> dsValues;
        for (int i = 0; i < DISTANCE_SENSOR_NUMBER; ++i)
        {
            dsValues[i] = ds[i]->getValue();
        }
        // following the external wall
        bool external_right_turn =
            dsValues[0] < 0.5 * OBSTACLE_THRESHOLD &&   // detecting NO right wall
            (dsValues[1] > 0.75 * OBSTACLE_THRESHOLD || // detecting wall in front
             dsValues[2] > 0.75 * OBSTACLE_THRESHOLD) &&
            dsValues[3] > 0.5 * OBSTACLE_THRESHOLD; // detecting left wall
        bool external_left_turn =
            dsValues[0] > 0.5 * OBSTACLE_THRESHOLD &&   // detecting right wall
            (dsValues[1] > 0.75 * OBSTACLE_THRESHOLD || // detecting wall in front
             dsValues[2] > 0.75 * OBSTACLE_THRESHOLD) &&
            dsValues[3] < 0.5 * OBSTACLE_THRESHOLD;
        // following the internal wall
        bool internal_right_turn =
            dsValues[0] < 0.75 * OBSTACLE_THRESHOLD && // detecting NO right wall
            (dsValues[1] < 0.5 * OBSTACLE_THRESHOLD || // detecting NO wall in front
             dsValues[2] < 0.25 * OBSTACLE_THRESHOLD) &&
            (dsValues[3] > 0.25 * OBSTACLE_THRESHOLD && // maintaining left wall distance
             dsValues[3] < 0.75 * OBSTACLE_THRESHOLD);
        bool internal_left_turn =
            (dsValues[0] > 0.25 * OBSTACLE_THRESHOLD && // maintaining right wall distance
             dsValues[0] < 0.75 * OBSTACLE_THRESHOLD) &&
            (dsValues[1] < 0.25 * OBSTACLE_THRESHOLD || // detecting NO wall in front
             dsValues[2] < 0.5 * OBSTACLE_THRESHOLD) &&
            dsValues[3] < 0.75 * OBSTACLE_THRESHOLD; // detecting NO left wall
        // detecting the end of the maze
        bool maze_end =
            dsValues[0] > 0.75 * OBSTACLE_THRESHOLD &&
            dsValues[1] > 0.75 * OBSTACLE_THRESHOLD &&
            dsValues[2] > 0.75 * OBSTACLE_THRESHOLD &&
            dsValues[3] > 0.75 * OBSTACLE_THRESHOLD;

        // get image from camera and detect colour
        const unsigned char *image = camera->getImage();
        if (image)
        { // image may be NULL if Robot.synchronization is FALSE
            int red{0};
            int green{0};
            int blue{0};

            // analysing the image from camera
            for (int i = width / 3; i < 2 * width / 3; ++i)
            {
                for (int j = height / 2; j < 3 * height / 4; ++j)
                {
                    red += camera->imageGetRed(image, width, i, j);
                    blue += camera->imageGetBlue(image, width, i, j);
                    green += camera->imageGetGreen(image, width, i, j);
                }
            }

            if (mRobot.getName() == "ROBOT_RED")
            {
                if ((red > 3 * green) && (red > 3 * blue)) // detecting red & terminating
                {
                    if (maze_end)
                    {
                        leftMotor->setVelocity(0);
                        rightMotor->setVelocity(0);
                        std::cout << mRobotName << ": Reaching target position in wall-following mode at " << mRobot.getTime() << " s" << std::endl;
                        break;
                    }
                }
                else // navigate the maze
                {
                    if (external_right_turn)
                    {
                        leftMotor->setVelocity(MAX_MOTOR_SPEED);
                        rightMotor->setVelocity(-MAX_MOTOR_SPEED);
                    }
                    else if (internal_left_turn)
                    {
                        leftMotor->setVelocity(-MAX_MOTOR_SPEED);
                        rightMotor->setVelocity(MAX_MOTOR_SPEED);
                    }
                    else // go straight
                    {
                        leftMotor->setVelocity(MAX_MOTOR_SPEED);
                        rightMotor->setVelocity(MAX_MOTOR_SPEED);
                    }
                }
            }
            else if (mRobot.getName() == "ROBOT_BLUE")
            {
                if ((blue > 3 * red) && (blue > 3 * green)) // detecting blue & terminating
                {
                    if (maze_end)
                    {
                        leftMotor->setVelocity(0);
                        rightMotor->setVelocity(0);
                        std::cout << mRobotName << ": Reaching target position in wall-following mode at " << mRobot.getTime() << std::endl;
                        break;
                    }
                }
                else // navigate the maze
                {
                    if (internal_right_turn)
                    {
                        leftMotor->setVelocity(MAX_MOTOR_SPEED);
                        rightMotor->setVelocity(-MAX_MOTOR_SPEED);
                    }
                    else if (external_left_turn)
                    {
                        leftMotor->setVelocity(-MAX_MOTOR_SPEED);
                        rightMotor->setVelocity(MAX_MOTOR_SPEED);
                    }
                    else // go straight
                    {
                        leftMotor->setVelocity(MAX_MOTOR_SPEED);
                        rightMotor->setVelocity(MAX_MOTOR_SPEED);
                    }
                }
            }
        }
    }
}

void robotControl::shortestPath(std::array<webots::DistanceSensor *, DISTANCE_SENSOR_NUMBER> ds, int timeStep, webots::Robot &robot)
{
    std::cout << mRobotName << ": Starting shortest-time mode at " << mRobot.getTime() << " s" << std::endl; // print message

    const auto startTime = mRobot.getTime(); // initialise start time

    // enable motors
    webots::Motor *leftMotor = mRobot.getMotor("left wheel motor");
    webots::Motor *rightMotor = mRobot.getMotor("right wheel motor");

    // get the camera device and enable cameras
    webots::Camera *camera = mRobot.getCamera("camera");
    camera->enable(timeStep);
    int width = camera->getWidth();
    int height = camera->getHeight();

    // main while loop
    while (mRobot.step(timeStep) != -1)
    {
        // intialise time tracking
        auto timeDiff = mRobot.getTime() - startTime;

        // read sensors outputs
        std::array<double, DISTANCE_SENSOR_NUMBER> dsValues;
        for (int i = 0; i < DISTANCE_SENSOR_NUMBER; ++i)
        {
            dsValues[i] = ds[i]->getValue();
        }

        bool maze_end = // detecting the end of the maze
            dsValues[0] > 0.75 * OBSTACLE_THRESHOLD &&
            dsValues[1] > 0.75 * OBSTACLE_THRESHOLD &&
            dsValues[2] > 0.75 * OBSTACLE_THRESHOLD &&
            dsValues[3] > 0.75 * OBSTACLE_THRESHOLD;

        // get image from camera and detect colour
        const unsigned char *image = camera->getImage();
        if (image)
        { // image may be NULL if Robot.synchronization is FALSE
            int red{0};
            int green{0};
            int blue{0};

            // analysing the image from camera
            for (int i = width / 3; i < 2 * width / 3; ++i)
            {
                for (int j = height / 2; j < 3 * height / 4; ++j)
                {
                    red += camera->imageGetRed(image, width, i, j);
                    blue += camera->imageGetBlue(image, width, i, j);
                    green += camera->imageGetGreen(image, width, i, j);
                }
            }

            if ((blue > 3 * red) && (blue > 3 * green)) // detecting blue & terminating
            {
                if (maze_end)
                {
                    leftMotor->setVelocity(0);
                    rightMotor->setVelocity(0);
                    std::cout << mRobotName << ": Reaching target position in shortest-time mode at " << mRobot.getTime() << " s" << std::endl;
                    break;
                }
            }
            else // navigate the maze
            {
                if (timeDiff < 1.5)
                { // go straight
                    leftMotor->setVelocity(MAX_MOTOR_SPEED);
                    rightMotor->setVelocity(MAX_MOTOR_SPEED);
                }
                else if (timeDiff >= 1.5 && timeDiff < 2)
                { // turn right
                    leftMotor->setVelocity(MAX_MOTOR_SPEED);
                    rightMotor->setVelocity(-MAX_MOTOR_SPEED);
                }
                else if (timeDiff >= 2 && timeDiff < 4.3)
                { // go straight
                    leftMotor->setVelocity(MAX_MOTOR_SPEED);
                    rightMotor->setVelocity(MAX_MOTOR_SPEED);
                }
                else if (timeDiff >= 4.3 && timeDiff < 4.8625)
                { // turn right
                    leftMotor->setVelocity(MAX_MOTOR_SPEED);
                    rightMotor->setVelocity(-MAX_MOTOR_SPEED);
                }
                else if (timeDiff >= 4.8625 && timeDiff < 6.8)
                { // go straight
                    leftMotor->setVelocity(MAX_MOTOR_SPEED);
                    rightMotor->setVelocity(MAX_MOTOR_SPEED);
                }
                else if (timeDiff >= 6.8 && timeDiff < 7.258)
                { // reorient for diagonal turning
                    leftMotor->setVelocity(MAX_MOTOR_SPEED);
                    rightMotor->setVelocity(-MAX_MOTOR_SPEED);
                }
                else if (timeDiff >= 7.258 && timeDiff < 11.5)
                { // go straight
                    leftMotor->setVelocity(MAX_MOTOR_SPEED);
                    rightMotor->setVelocity(MAX_MOTOR_SPEED);
                }
                else if (timeDiff >= 11.5 && timeDiff < 11.55)
                { // reorient for internal left turn
                    leftMotor->setVelocity(MAX_MOTOR_SPEED);
                    rightMotor->setVelocity(-MAX_MOTOR_SPEED);
                }
                else if (timeDiff >= 11.55 && timeDiff < 13)
                { // go straight
                    leftMotor->setVelocity(MAX_MOTOR_SPEED);
                    rightMotor->setVelocity(MAX_MOTOR_SPEED);
                }
                else if (timeDiff >= 13 && timeDiff < 13.55)
                { // turn left
                    leftMotor->setVelocity(-MAX_MOTOR_SPEED);
                    rightMotor->setVelocity(MAX_MOTOR_SPEED);
                }
                else if (timeDiff >= 13.55 && timeDiff < 15.7)
                { // go straight
                    leftMotor->setVelocity(MAX_MOTOR_SPEED);
                    rightMotor->setVelocity(MAX_MOTOR_SPEED);
                }
                else if (timeDiff >= 15.7 && timeDiff < 16.2)
                { // turn left
                    leftMotor->setVelocity(-MAX_MOTOR_SPEED);
                    rightMotor->setVelocity(MAX_MOTOR_SPEED);
                }
                else if (timeDiff >= 16.2 && timeDiff < 18.6)
                { // go straight
                    leftMotor->setVelocity(MAX_MOTOR_SPEED);
                    rightMotor->setVelocity(MAX_MOTOR_SPEED);
                }
                else if (timeDiff >= 18.6 && timeDiff < 18.73)
                { // reorient for diagonal finish
                    leftMotor->setVelocity(-MAX_MOTOR_SPEED);
                    rightMotor->setVelocity(MAX_MOTOR_SPEED);
                }
                else
                { // go straight
                    leftMotor->setVelocity(MAX_MOTOR_SPEED);
                    rightMotor->setVelocity(MAX_MOTOR_SPEED);
                }
            }
        }
    }
}

std::vector<std::vector<double>> robotControl::readRobotData(std::vector<double> &virtualTime, std::vector<double> &rawReadingDS0,
                                                             std::vector<double> &rawReadingDS1, std::vector<double> &rawReadingDS2,
                                                             std::vector<double> &rawReadingDS3) const
{
    // create an empty 2D vector of double for storing the information
    std::vector<std::vector<double>> data;

    std::ifstream fin{mFileName, std::ios::in};

    if (fin.is_open())
    { // check if file is open
        std::string fileLine;

        while (std::getline(fin, fileLine))
        { // while loop to get a line from the csv file

            std::stringstream fileRead{fileLine};
            std::vector<double> lineCopy; // vector to read lines from csv

            while (fileRead.good())
            { // check if opened successfully
                std::string lineData;
                std::getline(fileRead, lineData, ','); // reading dataline from file, delimited by ','
                std::stringstream doubleData{lineData};

                double elem;
                doubleData >> elem;
                lineCopy.push_back(elem);
            }

            if (mRobot.getName() == "ROBOT_RED") // case for robot red
            {
                for (size_t i = 0; i < lineCopy.size(); i++)
                {
                    if (i == 0 || i % 5 == 0)
                    {
                        virtualTime.push_back(lineCopy[i]);
                    }
                    else if (i == 1 || i % 5 == 1)
                    {
                        rawReadingDS0.push_back(lineCopy[i]);
                    }
                    else if (i == 2 || i % 5 == 2)
                    {
                        rawReadingDS1.push_back(lineCopy[i]);
                    }
                    else if (i == 3 || i % 5 == 3)
                    {
                        rawReadingDS2.push_back(lineCopy[i]);
                    }
                    else if (i == 4 || i % 5 == 4)
                    {
                        rawReadingDS3.push_back(lineCopy[i]);
                    }
                }
            }
            else // case for robot blue
            {
                data.push_back(lineCopy);
            }
        }

        // data read message
        std::cout << mRobotName << ": Reading data from " << mFileName << " succeeded!\n"; // data read message
    }
    else
    { // throw an error message
        throw std::runtime_error(mRobotName + ": Reading data from " + mFileName + " failed!");
    }

    return data;
}

// ----------------------------------------------------------------------------------------------- //
// MAIN FUNCION

int main(void)
{
    webots::Robot robot;                          // create a Robot instance
    webots::Robot &robotRef = robot;              // create reference to robot
    int timeStep = (int)robot.getBasicTimeStep(); // intialise timeStep

    std::string robotName = robot.getName();            // get name of the robot
    std::string fileName = robotName + ".csv";          // create file name using the robot
    std::string sortedFile = robotName + "_sorted.csv"; // file name for sorted blue data

    // get the distance sensors and enable
    std::array<webots::DistanceSensor *, DISTANCE_SENSOR_NUMBER> ds;
    std::array<std::string, DISTANCE_SENSOR_NUMBER> dsNames{
        "ds0", "ds1", "ds2", "ds3"};

    // read sensor input
    for (int i = 0; i < DISTANCE_SENSOR_NUMBER; ++i)
    {
        ds[i] = robot.getDistanceSensor(dsNames[i]);
        ds[i]->enable(timeStep);
    }

    // get the motor devices and initialise
    webots::Motor *leftMotor = robot.getMotor("left wheel motor");
    webots::Motor *rightMotor = robot.getMotor("right wheel motor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    // get keyboard and enable
    webots::Keyboard *keyboard = robot.getKeyboard();
    keyboard->enable(timeStep);
    int key{-1}; // initialise key with -1 (no key input)

    taskManager taskManager("TASK_MANAGER");                  // load class with TASK_MANAGER
    robotRed robotRed(robotName);                             // load ROBOT_RED with name and file name
    robotBlue robotBlue(robotName, sortedFile);               // load ROBOT_BLUE with name and file name
    robotControl robotControl(robotName, fileName, robotRef); // load robotControl with webots robot reference

    taskManager.printCommandList(); // print TASK_MANAGER command list

    while (robot.step(timeStep) != -1) // to get first key press for user selection from command list
    {
        const int prevKey = key;
        key = keyboard->getKey();

        // read sensors outputs (initialise)
        std::array<double, DISTANCE_SENSOR_NUMBER> dsValues;
        for (int i = 0; i < DISTANCE_SENSOR_NUMBER; ++i)
        {
            dsValues[i] = ds[i]->getValue();
        }

        if (key != prevKey) // reduces inadvertent toggling
        {
            if (key == '1') // run ROBOT_RED manually and write data to ROBOT_RED.csv
            {
                if (robot.getName() == "ROBOT_RED")
                {
                    std::cout << "TASK_MANAGER: Your input was " << key - 48 << " - now run ROBOT_RED in manual-control mode and write data to ROBOT_RED.csv" << std::endl;

                    robotRed.printRedCommands(); // print RED_ROBOT command list

                    while (robot.step(timeStep) != -1)
                    {
                        const int prevKey = key; // reset key value
                        key = keyboard->getKey();

                        if (robot.step(timeStep) % 5 == 0) // interval for writing data
                        {
                            std::vector<double> dataLine; // use a vector of double to store the data

                            // read sensors outputs (refresh)
                            std::array<double, DISTANCE_SENSOR_NUMBER> dsValues;
                            for (int i = 0; i < DISTANCE_SENSOR_NUMBER; ++i)
                            {
                                dsValues[i] = ds[i]->getValue();
                            }

                            robotControl.loadRobotData(dsValues, dataLine, robotRef);

                            try
                            {
                                robotControl.writeRobotFile(dataLine); // write new line to csv file
                            }
                            catch (const std::runtime_error &e) // error message for debugging
                            {
                                std::cerr << e.what() << std::endl;
                            }
                        }
                        // checking for keyboard input
                        if (key != prevKey)
                        {
                            if (key == 'W') // forward
                            {
                                robotControl.robotForward(); // call robot forward function
                            }
                            else if (key == 'S') // backward
                            {
                                robotControl.robotBackward(); // call robot backward function
                            }
                            else if (key == 'A') // left
                            {
                                robotControl.robotLeft(); // call robot left function
                            }
                            else if (key == 'D') // right
                            {
                                robotControl.robotRight(); // call robot right function
                            }
                            else if (key == ' ') // stop
                            {
                                robotControl.robotStop(); // call robot stop function
                            }
                        }
                    }
                }
                else
                {
                    std::cout << "You aren't ROBOT_RED" << std::endl;
                }
                break;
            }
            else if (key == '2') // run ROBOT_BLUE manually and write data to ROBOT_BLUE.csv
            {
                if (robot.getName() == "ROBOT_BLUE")
                {
                    std::cout << "TASK_MANAGER: Your input was " << key - 48 << " - now run ROBOT_BLUE in manual-control mode and write data to ROBOT_BLUE.csv" << std::endl;
                    robotBlue.printBlueCommands(); // print RED_ROBOT command list

                    while (robot.step(timeStep) != -1)
                    {
                        const int prevKey = key; // reset key value
                        key = keyboard->getKey();

                        if (robot.step(timeStep) % 5 == 0) // interval for writing data
                        {
                            std::vector<double> dataLine; // use a vector of double to store the data

                            // read sensors outputs (refresh)
                            std::array<double, DISTANCE_SENSOR_NUMBER> dsValues;
                            for (int i = 0; i < DISTANCE_SENSOR_NUMBER; ++i)
                            {
                                dsValues[i] = ds[i]->getValue();
                            }

                            robotControl.loadRobotData(dsValues, dataLine, robotRef);

                            try
                            {
                                robotControl.writeRobotFile(dataLine); // write new line to csv file
                            }
                            catch (const std::runtime_error &e) // error message for debugging
                            {
                                std::cerr << e.what() << std::endl;
                            }
                        }
                        // checking for keyboard input
                        if (key != prevKey)
                        {
                            if (key == webots::Keyboard::UP) // forward
                            {
                                robotControl.robotForward(); // call robot forward function
                            }
                            else if (key == webots::Keyboard::DOWN) // backward
                            {
                                robotControl.robotBackward(); // call robot backward function
                            }
                            else if (key == webots::Keyboard::LEFT) // left
                            {
                                robotControl.robotLeft(); // call robot left function
                            }
                            else if (key == webots::Keyboard::RIGHT) // right
                            {
                                robotControl.robotRight(); // call robot right function
                            }
                            else if (key == ' ') // stop
                            {
                                robotControl.robotStop(); // call robot stop function
                            }
                        }
                    }
                }
                else
                {
                    std::cout << "You aren't ROBOT_BLUE" << std::endl;
                }
                break;
            }
            else if (key == '3') // run ROBOT_RED and ROBOT_BLUE in wall_following
            {
                std::cout << "TASK_MANAGER: Your input was " << key - 48 << " - now run ROBOT_RED and ROBOT_BLUE in wall-following mode" << std::endl;
                robotControl.robotFollowWall(ds, timeStep); // call wall-following function
            }
            else if (key == '4') // run ROBOT_BLUE in shortest time mode
            {
                std::cout << "TASK_MANAGER: Your input was " << key - 48 << " - now run ROBOT_RED or ROBOT_BLUE in shortest-time mode" << std::endl;

                if (robot.getName() == "ROBOT_BLUE")
                {
                    robotControl.shortestPath(ds, timeStep, robotRef); // call function
                }
                else if (robot.getName() == "ROBOT_RED")
                {
                    std::cout << "ROBOT_BLUE will show you how to do it." << std::endl;
                }
            }
            else if (key == '5') // read and process ROBOT_RED data
            {
                if (robot.getName() == "ROBOT_RED")
                {
                    std::cout << "TASK_MANAGER: Your input was " << key - 48 << " - now read data from ROBOT_RED.csv and process" << std::endl;

                    try
                    {
                        std::vector<double> virtualTime;   // vector to store the Virtual_Time
                        std::vector<double> rawReadingDS0; // vector to store Raw_Reading_ds0
                        std::vector<double> rawReadingDS1; // vector to store Raw_Reading_ds1
                        std::vector<double> rawReadingDS2; // vector to store Raw_Reading_ds2
                        std::vector<double> rawReadingDS3; // vector to store Raw_Reading_ds3
                        // std::vector<std::vector<double>> data; // create an empty 2D vector for storing all data

                        // read data from file
                        robotControl.readRobotData(virtualTime, rawReadingDS0, rawReadingDS1, rawReadingDS2, rawReadingDS3);

                        if (robot.getName() == "ROBOT_RED")
                        {
                            robotRed.robotRedSummary(virtualTime, rawReadingDS0, rawReadingDS1, rawReadingDS2, rawReadingDS3);
                        }
                        else
                        {
                            std::cout << "You're not ROBOT_RED" << std::endl;
                        }
                    }

                    catch (const std::runtime_error &e) // error message
                    {
                        std::cerr << e.what() << std::endl;
                    }
                }
                else
                {
                    std::cout << "You're not ROBOT_RED." << std::endl;
                }
            }
            else if (key == '6') // read and process ROBOT_BLUE data
            {
                if (robot.getName() == "ROBOT_BLUE")
                {
                    try
                    {
                        std::cout << "TASK_MANAGER: Your input was " << key - 48 << " - now read data from ROBOT_BLUE.csv and process" << std::endl;

                        std::vector<double> virtualTime;   // vector to store the Virtual_Time
                        std::vector<double> rawReadingDS0; // vector to store Raw_Reading_ds0
                        std::vector<double> rawReadingDS1; // vector to store Raw_Reading_ds1
                        std::vector<double> rawReadingDS2; // vector to store Raw_Reading_ds2
                        std::vector<double> rawReadingDS3; // vector to store Raw_Reading_ds3

                        // read data from file
                        auto data = robotControl.readRobotData(virtualTime, rawReadingDS0, rawReadingDS1, rawReadingDS2, rawReadingDS3);
                        auto data2 = robotBlue.sortData(data); // sort data
                        robotBlue.writeSortedData(data2);      // write out the data in a separate csv file
                    }
                    catch (const std::runtime_error &e) // error message
                    {
                        std::cerr << e.what() << std::endl;
                    }
                }
                else
                {
                    std::cout << "You're not ROBOT_BLUE." << std::endl;
                }
            }
        }
    }

    return 0;
}