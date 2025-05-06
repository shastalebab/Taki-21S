#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline ez::PID dunkerPID(2.6, 0.0, 0.8, 0.0, "dunker");

inline pros::Optical colorSens(11);
inline pros::Distance distanceSens(12);
inline pros::Optical ringSens(13);

inline pros::Motor intake(7);
inline pros::Motor dunker(8);
inline pros::Rotation dunkerSens(14);

inline ez::Piston mogomech('A');
inline ez::Piston doinker('C');
inline ez::Piston actuatedIntake('D');

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
extern Colors matchColor;

void setIntake(int speed);
void setIndexing();
void setDunker(int position);
void setDunker(int position, int max_speed);
void resetDunker();
void setMogo(bool state);
void primeMogo();
void setDoinker(bool state);
void getPos();
void setActuatedIntake(bool state);
void antiJamDisabled(bool state);
void setAlliance(Colors alliance);
void sendHaptic(string input);
void colorToggle();

void setIntakeOp();
void setDunkerOp();
void setMogoOp();
void setDoinkerOp();

void colorTask();
void mogoTask();
void dunkerTask();
void unjamTask();