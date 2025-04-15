#pragma once

#include "EZ-Template/api.hpp"


enum class AutonMode {
    ODOM = 0,
    PLAIN = 1,
    BRAIN = 2,
    DRIVER = 3
};

enum class MovementType {
    DRIVE = 0,
    TURN = 1,
    SWING = 2
};

enum class Wait {
    WAIT = 0,
    QUICK = 1,
    CHAIN = 2
};

class Coordinate {
    public: 
        double x;
        double y;
        double t;
        double speed;
        ez::drive_directions facing;
        MovementType movement;

        ez::e_swing side;
        ez::e_angle_behavior behavior;
        double main;
        double opp;
        double theta;
};

extern AutonMode autonMode;
extern vector<Coordinate> autonPath;

// Internal math
double getDistance(Coordinate point1, Coordinate point2, ez::drive_directions direction);
double getTheta(Coordinate point1, Coordinate point2, ez::drive_directions direction);
Coordinate getArc(Coordinate startpoint, double right, double left, double distance);
Coordinate getArcFromTheta(Coordinate startpoint, double right, double left, double theta);
std::vector<Coordinate> injectArc(Coordinate startpoint, ez::e_swing side, ez::e_angle_behavior behavior, double main, double opp, double theta, double lookAhead);
std::vector<Coordinate> injectPath(std::vector<Coordinate> coordList, double lookAhead);

// Set position wrappers
void setPosition(double x, double y);
void setPosition(double x, double y, double t);

// Wait wrappers
void pidWait(Wait type);
void pidWaitUntil(double distance);
void pidWaitUntil(Coordinate coordinate);

// Move to point wrappers
void moveToPoint(Coordinate newpoint, ez::drive_directions direction, int speed);
void moveToPoint(Coordinate currentpoint, Coordinate newpoint, ez::drive_directions direction, int speed);

// Drive set wrappers
void driveSet(double distance, int speed, bool slew);
void driveSet(double distance, int speed);

// Turn set wrappers
void turnSet(double theta, int speed);

// Swing set wrappers
void swingSet(ez::e_swing side, double theta, double main, double opp, ez::e_angle_behavior behavior);
void swingSet(ez::e_swing side, double theta, double main, ez::e_angle_behavior behavior);
void swingSet(ez::e_swing side, double theta, double main, double opp);
void swingSet(ez::e_swing side, double theta, double main);

// Print path
void getPath();
void getPathInjected();