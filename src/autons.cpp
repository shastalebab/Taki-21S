#include "main.h"  // IWYU pragma: keep

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
	// P, I, D, and Start I
	chassis.pid_drive_constants_set(20.0, 0.0, 100.0);		   // Fwd/rev constants, used for odom and non odom motions
	chassis.pid_heading_constants_set(11.0, 0.0, 25.0);		   // Holds the robot straight while going forward without odom
	chassis.pid_turn_constants_set(3.25, 0.05, 25.0, 15.0);	   // Turn in place constants
	chassis.pid_swing_constants_set(6.0, 0.0, 65.0);		   // Swing constants
	chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);	   // Angular control for odom motions
	chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

	// Exit conditions
	chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
	chassis.pid_swing_exit_condition_set(70_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
	chassis.pid_drive_exit_condition_set(70_ms, 2.2_in, 160_ms, 6_in, 200_ms, 200_ms);
	chassis.pid_odom_turn_exit_condition_set(40_ms, 3_deg, 170_ms, 6_deg, 500_ms, 750_ms);
	chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
	chassis.pid_turn_chain_constant_set(4_deg);
	chassis.pid_swing_chain_constant_set(5_deg);
	chassis.pid_drive_chain_constant_set(4_in);

	// Slew constants
	chassis.slew_turn_constants_set(3_deg, 70);
	chassis.slew_drive_constants_set(3_in, 70);
	chassis.slew_swing_constants_set(3_in, 80);

	// The amount that turns are prioritized over driving in odom motions
	// - if you have tracking wheels, you can run this higher.  1.0 is the max
	chassis.odom_turn_bias_set(0.9);

	chassis.odom_look_ahead_set(7_in);			 // This is how far ahead in the path the robot looks at
	chassis.odom_boomerang_distance_set(16_in);	 // This sets the maximum distance away from target that the carrot point can be
	chassis.odom_boomerang_dlead_set(0.625);	 // This handles how aggressive the end of boomerang motions are

	chassis.pid_angle_behavior_set(shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

//
// RED NEGATIVE
//

void red_solowp() {
	setAlliance(Colors::RED);
	setPosition(62.75, 18.5, 152);
	// score alliance and grab mogo
	driveSet(7, 90);
	setDunker(200);
	pidWait(Wait::WAIT);
	delayMillis(100);
	driveSet(-39.1, 90);
	primeMogo();
	pidWaitUntil(-29.5_in);
	resetDunker();
	setMogo(true);
	setIntake(127);
	pidWait(Wait::CHAIN);
	// sweep ring rush rings
	moveToPoint({35, 57}, fwd, 70);
	pidWait(Wait::CHAIN);
	moveToPoint({14, 59}, fwd, 70);
	pidWait(Wait::WAIT);
	delayMillis(100);
	swingSet(RIGHT_SWING, -65, 127, 60, cw);
	pidWait(Wait::CHAIN);
	// grab bottom ring of ring stack, and go to middle
	moveToPoint({28, 48}, fwd, 100);
	pidWait(Wait::WAIT);
	moveToPoint({72, 27}, fwd, 100);
	pidWait(Wait::QUICK);
	setMogo(false);
	// grab mogo
	moveToPoint({100, 48}, rev, 90);
	setIntake(0);
	delayMillis(500);
	primeMogo();
	pidWait(Wait::CHAIN);
	setIntake(127);
	// grab bottom ring and middle ring
	turnSet(90, 100);
	pidWait(Wait::CHAIN);
	driveSet(20, 100);
	pidWait(Wait::CHAIN);
	moveToPoint({76, 16}, fwd, 100);
	pidWait(Wait::CHAIN);
	// touch ladder or go to positive corner
	if(ladderBool) {
		moveToPoint({72, 48}, fwd, 80);
	} else {
		moveToPoint({128, 16}, fwd, 127);
		pidWait(Wait::CHAIN);
		turnSet(135, 100);
		pidWait(Wait::CHAIN);
		driveSet(24, 127);
	}
	pidWait(Wait::WAIT);
}

void red_7neg() {
	setAlliance(Colors::RED);
	setPosition(62.75, 18.5, 152);
	// score alliance and grab mogo
	driveSet(7, 90);
	setDunker(200);
	pidWait(Wait::WAIT);
	delayMillis(100);
	driveSet(-39.1, 90);
	primeMogo();
	pidWaitUntil(-29.5_in);
	resetDunker();
	setMogo(true);
	setIntake(127);
	pidWait(Wait::CHAIN);
	// sweep ring rush rings
	moveToPoint({35, 57}, fwd, 70);
	pidWait(Wait::CHAIN);
	moveToPoint({14, 59}, fwd, 70);
	pidWait(Wait::WAIT);
	delayMillis(100);
	swingSet(RIGHT_SWING, -65, 127, 60, cw);
	pidWait(Wait::CHAIN);
	// grab bottom ring of ring stack, then score corner
	moveToPoint({28, 48}, fwd, 100);
	pidWait(Wait::CHAIN);
	moveToPoint({28, 28}, fwd, 100);
	pidWait(Wait::CHAIN);
	turnSet(225, 127);
	setDunker(256);
	pidWait(Wait::CHAIN);
	// score corner
	driveSet(28, 70, true);
	pidWait(Wait::WAIT);
	driveSet(-12, 30);
	setDunker(0, 50);
	pidWait(Wait::CHAIN);
	// grab mid top ring
	turnSet(90, 60);
	pidWait(Wait::CHAIN);
	driveSet(58, 80, true);
	delayMillis(200);
	setIntake(0);
	setActuatedIntake(true);
	delayMillis(600);
	setIntake(127);
	pidWaitUntil(50_in);
	setDunker(12);
	setActuatedIntake(false);
	antiJamDisabled(true);
	pidWait(Wait::WAIT);
	delayMillis(200);
	// score on wallstake
	driveSet(-16, 60);
	pidWait(Wait::CHAIN);
	turnSet(309, 90);
	pidWait(Wait::CHAIN);
	setDunker(146);
	driveSet(60, 90);
	pidWait(Wait::WAIT);
}

void red_7greed() {
	setAlliance(Colors::RED);
	setPosition(62.75, 18.5, 152);
	// score alliance and grab mogo
	driveSet(7, 90);
	setDunker(200);
	pidWait(Wait::WAIT);
	delayMillis(100);
	driveSet(-39.1, 90);
	primeMogo();
	pidWaitUntil(-29.5_in);
	resetDunker();
	setMogo(true);
	setIntake(127);
	pidWait(Wait::CHAIN);
	// sweep ring rush rings
	moveToPoint({35, 57}, fwd, 70);
	pidWait(Wait::CHAIN);
	moveToPoint({14, 59}, fwd, 70);
	pidWait(Wait::WAIT);
	delayMillis(100);
	swingSet(RIGHT_SWING, -65, 127, 60, cw);
	pidWait(Wait::CHAIN);
	// grab bottom ring of ring stack, then score corner
	moveToPoint({28, 48}, fwd, 100);
	pidWait(Wait::CHAIN);
	moveToPoint({28, 28}, fwd, 100);
	pidWait(Wait::CHAIN);
	turnSet(225, 127);
	setDunker(256);
	pidWait(Wait::CHAIN);
	// score corner
	driveSet(28, 70, true);
	pidWait(Wait::WAIT);
	driveSet(-12, 30);
	setDunker(0, 50);
	pidWait(Wait::CHAIN);
	// grab mid top ring
	turnSet(90, 60);
	pidWait(Wait::CHAIN);
	driveSet(58, 80, true);
	delayMillis(200);
	setIntake(0);
	setActuatedIntake(true);
	delayMillis(600);
	setIntake(127);
	pidWaitUntil(50_in);
	resetDunker();
	setActuatedIntake(false);
	antiJamDisabled(true);
	pidWait(Wait::WAIT);
	delayMillis(200);
	// touch ladder or go to positive corner
	if(ladderBool) {
		driveSet(-16, 60);
		pidWait(Wait::CHAIN);
		turnSet(45, 90);
		pidWait(Wait::CHAIN);
		driveSet(20, 127);
		setDunker(200);
		pidWait(Wait::WAIT);
	} else {
		driveSet(48, 127);
		pidWait(Wait::CHAIN);
		turnSet(135, 100);
		pidWait(Wait::CHAIN);
		driveSet(24, 127);
	}
	pidWait(Wait::WAIT);
}

//
// RED POSITIVE
//

void red_goalrush() {
	setAlliance(Colors::RED); 
	setPosition(101.625, 19.75, 19);
	// grab ring and score on mogo
	driveSet(37, 127, true);
	pidWaitUntil(16_in);
	setDunker(256);
	setDunker(256);
	pidWait(Wait::WAIT);
	delayMillis(100);
	// grab other mogo
	swingSet(RIGHT_SWING, 90, 80, 10, cw);
	delayMillis(500);
	driveSet(-10, 80);
	pidWait(Wait::CHAIN);
	setMogo(true);
	// go to corner
	moveToPoint({121, 24}, fwd, 100);
	setIntake(127);
	pidWait(Wait::WAIT);
	turnSet(135, 90);
	pidWait(Wait::CHAIN);
	// score corner
	driveSet(28, 60, true);
	pidWait(Wait::WAIT);
	setPosition(134, 10);
	driveSet(-12, 30);
	setDunker(170, 50);
	setDunker(170, 50);
	pidWait(Wait::CHAIN);
	// grab mid top ring
	turnSet(-90, 60);
	pidWait(Wait::CHAIN);
	setActuatedIntake(true);
	driveSet(58, 80, true);
	pidWaitUntil(48_in);
	setActuatedIntake(false);
	antiJamDisabled(true);
	pidWait(Wait::WAIT);
	delayMillis(200);
	// score alliance stake
	driveSet(-3, 60);
	pidWait(Wait::CHAIN);
	antiJamDisabled(false);
	turnSet(180, 70);
	pidWait(Wait::CHAIN);
	setDunker(220, 50);
	setDunker(220, 50);
	driveSet(5, 90);
	pidWait(Wait::QUICK);
	driveSet(-3, 90);
	pidWait(Wait::CHAIN);
	// score bottom ring on wallstake
	moveToPoint({124, 44}, fwd, 100);
	delayMillis(500);
	setDunker(12);
	pidWait(Wait::WAIT);
	moveToPoint({138, 60}, fwd, 60);
	pidWait(Wait::WAIT);
	setDunker(146);
}

void red_5pos() {
	setAlliance(Colors::RED);
	setPosition(81.25, 18.5, 208);
	// score alliance and grab mogo
	driveSet(7, 90);
	setDunker(200);
	pidWait(Wait::WAIT);
	delayMillis(100);
	driveSet(-39.1, 90);
	primeMogo();
	pidWaitUntil(-29.5_in);
	resetDunker();
	setMogo(true);
	pidWait(Wait::CHAIN);
	// grab middle rings with doinker
	moveToPoint({81, 52}, fwd, 90);
	pidWait(Wait::QUICK);
	turnSet(-5, 90);
	pidWait(Wait::QUICK);
	driveSet(4, 90);
	setDoinker(true);
	pidWait(Wait::QUICK);
	// bring rings back and score all of them
	driveSet(-40, 90);
	pidWait(Wait::CHAIN);
	setIntake(127);
	turnSet({120, 48}, fwd, 90);
	pidWait(Wait::CHAIN);
	setDoinker(false);
	driveSet(48, 90);
	// score corner
	pidWait(Wait::WAIT);
	turnSet(180, 100);
	pidWait(Wait::CHAIN);
	driveSet(22, 100);
	pidWait(Wait::CHAIN);
	turnSet(135, 127);
	setDunker(256);
	pidWait(Wait::CHAIN);
	// score corner
	driveSet(28, 60, true);
	pidWait(Wait::WAIT);
	setPosition(134, 10);
	driveSet(-12, 30);
	setDunker(0, 50);
	pidWait(Wait::CHAIN);
	// grab mid top ring
	turnSet(-90, 60);
	pidWait(Wait::CHAIN);
	driveSet(58, 80, true);
	delayMillis(200);
	setIntake(0);
	setActuatedIntake(true);
	delayMillis(600);
	setIntake(127);
	pidWaitUntil(50_in);
	resetDunker();
	setActuatedIntake(false);
	antiJamDisabled(true);
	pidWait(Wait::WAIT);
	delayMillis(200);
	if(ladderBool) {
		driveSet(-16, 60);
		pidWait(Wait::CHAIN);
		turnSet(-45, 90);
		pidWait(Wait::CHAIN);
		driveSet(20, 127);
		setDunker(200);
		pidWait(Wait::WAIT);
	} else {
		driveSet(-12, 127);
		pidWait(Wait::CHAIN);
		turnSet(225, 100);
		pidWait(Wait::CHAIN);
		driveSet(-60, 127);
	}
	pidWait(Wait::WAIT);
}

//
// BLUE NEGATIVE
//

void blue_solowp() {
	setAlliance(Colors::BLUE);
	setPosition(81.25, 18.5, 208);
	// score alliance and grab mogo
	driveSet(7, 90);
	setDunker(200);
	pidWait(Wait::WAIT);
	delayMillis(100);
	driveSet(-39.1, 90);
	primeMogo();
	pidWaitUntil(-29.5_in);
	resetDunker();
	setMogo(true);
	setIntake(127);
	pidWait(Wait::CHAIN);
	// sweep ring rush rings
	moveToPoint({109, 57}, fwd, 70);
	pidWait(Wait::CHAIN);
	moveToPoint({130, 59}, fwd, 70);
	pidWait(Wait::WAIT);
	delayMillis(100);
	swingSet(LEFT_SWING, 65, 127, 60, ccw);
	pidWait(Wait::CHAIN);
	// grab bottom ring of ring stack, and go to middle
	moveToPoint({116, 48}, fwd, 100);
	pidWait(Wait::WAIT);
	moveToPoint({72, 27}, fwd, 100);
	pidWait(Wait::QUICK);
	setMogo(false);
	// grab mogo
	moveToPoint({44, 48}, rev, 90);
	setIntake(0);
	delayMillis(500);
	primeMogo();
	pidWait(Wait::CHAIN);
	setIntake(127);
	// grab bottom ring and middle ring
	turnSet(-90, 100);
	pidWait(Wait::CHAIN);
	driveSet(20, 100);
	pidWait(Wait::CHAIN);
	moveToPoint({68, 16}, fwd, 100);
	pidWait(Wait::CHAIN);
	// touch ladder
	if(ladderBool) {
		moveToPoint({72, 48}, fwd, 80);
	} else {
		moveToPoint({16, 16}, fwd, 127);
		pidWait(Wait::CHAIN);
		turnSet(225, 100);
		pidWait(Wait::CHAIN);
		driveSet(24, 127);
	}
	pidWait(Wait::WAIT);
}

void blue_7neg() {
	setAlliance(Colors::BLUE);
	setPosition(81.25, 18.5, 208);
	// score alliance and grab mogo
	driveSet(7, 90);
	setDunker(200);
	pidWait(Wait::WAIT);
	delayMillis(100);
	driveSet(-39.1, 90);
	primeMogo();
	pidWaitUntil(-29.5_in);
	resetDunker();
	setMogo(true);
	setIntake(127);
	pidWait(Wait::CHAIN);
	// sweep ring rush rings
	moveToPoint({109, 57}, fwd, 70);
	pidWait(Wait::CHAIN);
	moveToPoint({130, 59}, fwd, 70);
	pidWait(Wait::WAIT);
	delayMillis(100);
	swingSet(LEFT_SWING, 65, 127, 60, ccw);
	pidWait(Wait::CHAIN);
	// grab bottom ring of ring stack, then score corner
	moveToPoint({116, 48}, fwd, 100);
	pidWait(Wait::CHAIN);
	moveToPoint({116, 28}, fwd, 100);
	pidWait(Wait::CHAIN);
	turnSet(135, 127);
	setDunker(256);
	pidWait(Wait::CHAIN);
	// score corner
	driveSet(28, 70, true);
	pidWait(Wait::WAIT);
	driveSet(-12, 30);
	setDunker(0, 50);
	pidWait(Wait::CHAIN);
	// grab mid top ring
	turnSet(-90, 60);
	pidWait(Wait::CHAIN);
	driveSet(58, 80, true);
	delayMillis(200);
	setIntake(0);
	setActuatedIntake(true);
	delayMillis(600);
	setIntake(127);
	pidWaitUntil(50_in);
	setDunker(12);
	setActuatedIntake(false);
	antiJamDisabled(true);
	pidWait(Wait::WAIT);
	delayMillis(200);
	// score on wallstake
	driveSet(-16, 60);
	pidWait(Wait::CHAIN);
	turnSet(51, 90);
	pidWait(Wait::CHAIN);
	setDunker(146);
	driveSet(60, 90);
	pidWait(Wait::WAIT);
}

void blue_7greed() {
	setAlliance(Colors::BLUE);
	setPosition(81.25, 18.5, 208);
	// score alliance and grab mogo
	driveSet(7, 90);
	setDunker(200);
	pidWait(Wait::WAIT);
	delayMillis(100);
	driveSet(-39.1, 90);
	primeMogo();
	pidWaitUntil(-29.5_in);
	resetDunker();
	setMogo(true);
	setIntake(127);
	pidWait(Wait::CHAIN);
	// sweep ring rush rings
	moveToPoint({109, 57}, fwd, 70);
	pidWait(Wait::CHAIN);
	moveToPoint({130, 59}, fwd, 70);
	pidWait(Wait::WAIT);
	delayMillis(100);
	swingSet(LEFT_SWING, 65, 127, 60, ccw);
	pidWait(Wait::CHAIN);
	// grab bottom ring of ring stack, then score corner
	moveToPoint({116, 48}, fwd, 100);
	pidWait(Wait::CHAIN);
	moveToPoint({116, 28}, fwd, 100);
	pidWait(Wait::CHAIN);
	turnSet(135, 127);
	setDunker(256);
	pidWait(Wait::CHAIN);
	// score corner
	driveSet(28, 70, true);
	pidWait(Wait::WAIT);
	driveSet(-12, 30);
	setDunker(0, 50);
	pidWait(Wait::CHAIN);
	// grab mid top ring
	turnSet(-90, 60);
	pidWait(Wait::CHAIN);
	driveSet(58, 80, true);
	delayMillis(200);
	setIntake(0);
	setActuatedIntake(true);
	delayMillis(600);
	setIntake(127);
	pidWaitUntil(50_in);
	resetDunker();
	setActuatedIntake(false);
	antiJamDisabled(true);
	pidWait(Wait::WAIT);
	delayMillis(200);
	// touch ladder or go to positive corner
	if(ladderBool) {
		driveSet(-16, 60);
		pidWait(Wait::CHAIN);
		turnSet(-45, 90);
		pidWait(Wait::CHAIN);
		driveSet(20, 127);
		setDunker(200);
		pidWait(Wait::WAIT);
	} else {
		driveSet(48, 127);
		pidWait(Wait::CHAIN);
		turnSet(225, 100);
		pidWait(Wait::CHAIN);
		driveSet(24, 127);
	}
	pidWait(Wait::WAIT);
}

//
// BLUE POSITIVE
//

void blue_goalrush() { 
	setAlliance(Colors::BLUE); 
	setPosition(42.375, 19.75, 341);
	// grab ring and score on mogo
	driveSet(37, 127, true);
	pidWaitUntil(16_in);
	setDunker(256);
	setDunker(256);
	pidWait(Wait::WAIT);
	delayMillis(100);
	// grab other mogo
	swingSet(LEFT_SWING, -90, 80, 10, ccw);
	delayMillis(500);
	driveSet(-10, 80);
	pidWait(Wait::CHAIN);
	setMogo(true);
	// go to corner
	moveToPoint({23, 24}, fwd, 100);
	setIntake(127);
	pidWait(Wait::WAIT);
	turnSet(225, 90);
	pidWait(Wait::CHAIN);
	// score corner
	driveSet(28, 60, true);
	pidWait(Wait::WAIT);
	setPosition(10, 10);
	driveSet(-12, 30);
	setDunker(170, 50);
	setDunker(170, 50);
	pidWait(Wait::CHAIN);
	// grab mid top ring
	turnSet(90, 60);
	pidWait(Wait::CHAIN);
	setActuatedIntake(true);
	driveSet(58, 80, true);
	pidWaitUntil(48_in);
	setActuatedIntake(false);
	antiJamDisabled(true);
	pidWait(Wait::WAIT);
	delayMillis(200);
	// score alliance stake
	driveSet(-3, 60);
	pidWait(Wait::CHAIN);
	antiJamDisabled(false);
	turnSet(-180, 70);
	pidWait(Wait::CHAIN);
	setDunker(220, 50);
	setDunker(220, 50);
	driveSet(5, 90);
	pidWait(Wait::QUICK);
	driveSet(-3, 90);
	pidWait(Wait::CHAIN);
	// score bottom ring on wallstake
	moveToPoint({20, 44}, fwd, 100);
	delayMillis(500);
	setDunker(12);
	pidWait(Wait::WAIT);
	moveToPoint({6, 60}, fwd, 60);
	pidWait(Wait::WAIT);
	setDunker(146);
}

void blue_5pos() {
	setAlliance(Colors::BLUE);
	setPosition(62.75, 18.5, 152);
	// score alliance and grab mogo
	driveSet(7, 90);
	setDunker(200);
	pidWait(Wait::WAIT);
	delayMillis(100);
	driveSet(-39.1, 90);
	primeMogo();
	pidWaitUntil(-29.5_in);
	resetDunker();
	setMogo(true);
	pidWait(Wait::CHAIN);
	// grab middle rings with doinker
	moveToPoint({66, 58}, fwd, 90);
	pidWait(Wait::QUICK);
	setDoinker(true);
	// bring rings back and score all of them
	swingSet(LEFT_SWING, -10, 80, 50, ccw);
	pidWait(Wait::CHAIN);
	setDoinker(false);
	setIntake(127);
	swingSet(RIGHT_SWING, -90, 127, 43, ccw);
	pidWait(Wait::CHAIN);
	driveSet(14, 90);
	// score corner
	pidWait(Wait::WAIT);
	turnSet(180, 100);
	pidWait(Wait::CHAIN);
	driveSet(18, 100);
	setDunker(256);
	pidWait(Wait::CHAIN);
	turnSet(225, 127);
	pidWait(Wait::CHAIN);
	// score corner
	driveSet(28, 60, true);
	pidWait(Wait::WAIT);
	setPosition(10, 10);
	driveSet(-12, 30);
	setDunker(0, 50);
	pidWait(Wait::CHAIN);
	// grab mid top ring
	turnSet(90, 60);
	pidWait(Wait::CHAIN);
	driveSet(58, 80, true);
	delayMillis(200);
	setIntake(0);
	setActuatedIntake(true);
	delayMillis(600);
	setIntake(127);
	pidWaitUntil(50_in);
	resetDunker();
	setActuatedIntake(false);
	antiJamDisabled(true);
	pidWait(Wait::WAIT);
	delayMillis(200);
	if(ladderBool) {
		driveSet(-16, 60);
		pidWait(Wait::CHAIN);
		turnSet(45, 90);
		pidWait(Wait::CHAIN);
		driveSet(20, 127);
		setDunker(200);
		pidWait(Wait::WAIT);
	} else {
		driveSet(-12, 127);
		pidWait(Wait::CHAIN);
		turnSet(135, 100);
		pidWait(Wait::CHAIN);
		driveSet(-60, 127);
	}
	pidWait(Wait::WAIT);
}

//
// DEFAULT / TEST AUTONS
//

void move_forward() {
	driveSet(24, DRIVE_SPEED, true);
	pidWait(Wait::WAIT);
}

void tuning() {
	setAlliance(Colors::RED);
	resetDunker();
	setIntake(127);
	primeMogo();
	driveSet(24, 60);
}

void red_testauto() {
	setAlliance(Colors::RED);
	resetDunker();
	setActuatedIntake(true);
	setIntake(127);
	primeMogo();
}

void blue_testauto() {
	setAlliance(Colors::BLUE);
	resetDunker();
	setIntake(127);
	primeMogo();
}