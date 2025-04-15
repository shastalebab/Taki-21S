#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline ez::PID dunkerPID(1.35, .04, .75, 0, "dunker");

inline pros::Optical colorSens(11);
inline pros::Distance distanceSens(19);
inline pros::adi::LineSensor hookSens('B');

inline pros::Motor intake(7);
inline pros::Motor dunker(8);

inline ez::Piston mogomech('A');

void setIntake(int speed);
void setDunker(int position);
void setMogo(bool state);

void setIntakeOp();
void setDunkerOp();
void setMogoOp();

void colorTask();
void mogoTask();
void dunkerTask();
void unjamTask();

enum class Colors {
    RED = 0,
    BLUE = 1,
    NEUTRAL = 2
};

enum class AutoMogo {
    OFF = 0,
    PRIMED = 1
};

extern Colors allianceColor;
extern AutoMogo mogoState;