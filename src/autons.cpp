#include "autons.hpp"

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "drive.hpp"
#include "main.h"  // IWYU pragma: keep
#include "subsystems.hpp"

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
	chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
	chassis.pid_drive_exit_condition_set(50_ms, 2_in, 160_ms, 6_in, 200_ms, 200_ms);
	chassis.pid_odom_turn_exit_condition_set(50_ms, 4_deg, 170_ms, 8_deg, 500_ms, 750_ms);
	chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
	chassis.pid_turn_chain_constant_set(4_deg);
	chassis.pid_swing_chain_constant_set(5_deg);
	chassis.pid_drive_chain_constant_set(6_in);

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
	allianceColor = Colors::RED;
	setPosition(62.75, 18.5, 152);
	// score alliance and grab mogo
	driveSet(6.5, 90);
	setDunker(2000);
	pidWait(Wait::WAIT);
	driveSet(-38.6, 90);
	primeMogo();
	pidWaitUntil(-28.5_in);
	tareDunker();
	setMogo(true);
	setIntake(127);
	pidWait(Wait::CHAIN);
	// sweep ring rush rings
	moveToPoint({35, 57}, fwd, 70);
	pidWait(Wait::CHAIN);
	moveToPoint({14, 58}, fwd, 70);
	pidWait(Wait::WAIT);
	delayMillis(500);
	swingSet(RIGHT_SWING, -70, 127, 70, cw);
	pidWait(Wait::CHAIN);
	// grab bottom ring of ring stack, and go to middle
	moveToPoint({23, 42}, fwd, 100);
	pidWait(Wait::CHAIN);
	moveToPoint({72, 24}, fwd, 100);
	pidWait(Wait::QUICK);
	setMogo(false);
	// grab mogo
	moveToPoint({96, 48}, rev, 90);
	delayMillis(500);
	primeMogo();
	pidWait(Wait::CHAIN);
	// grab bottom ring and middle ring
	turnSet(90, 100);
	pidWait(Wait::CHAIN);
	driveSet(20, 100);
	pidWait(Wait::CHAIN);
	moveToPoint({76, 16}, fwd, 100);
	pidWait(Wait::CHAIN);
	// touch ladder or go to positive corner
	if(ladderBool) {
		swingSet(LEFT_SWING, 135, 127, 50, ccw);
	} else {
		moveToPoint({128, 16}, fwd, 127);
		pidWait(Wait::CHAIN);
		turnSet(135, 100);
		pidWait(Wait::CHAIN);
		driveSet(24, 127);
	}
	pidWait(Wait::WAIT);
}

void red_6neg() {
	allianceColor = Colors::RED;
	setPosition(62.75, 18.5, 152);
	// grab mogo
	driveSet(-32, 90);
	primeMogo();
	pidWaitUntil(-22_in);
	setMogo(true);
	setIntake(127);
	pidWait(Wait::CHAIN);
	// sweep ring rush rings
	moveToPoint({35, 57}, fwd, 70);
	pidWait(Wait::CHAIN);
	moveToPoint({14, 58}, fwd, 70);
	pidWait(Wait::WAIT);
	delayMillis(500);
	swingSet(RIGHT_SWING, -70, 127, 70, cw);
	pidWait(Wait::CHAIN);
	// grab bottom ring of ring stack, then score corner
	moveToPoint({23, 42}, fwd, 100);
	pidWait(Wait::CHAIN);
	moveToPoint({23, 22}, fwd, 100);
	pidWait(Wait::CHAIN);
	turnSet(225, 127);
	pidWait(Wait::CHAIN);
	// score corner
	driveSet(24, 127, true);
	pidWait(Wait::CHAIN);
	driveSet(-16, 127);
	pidWait(Wait::CHAIN);
	driveSet(24, 127, true);
	pidWait(Wait::CHAIN);
	driveSet(-12, 127);
	pidWait(Wait::CHAIN);
	// grab mid top ring
	turnSet(90, 100);
	pidWait(Wait::CHAIN);
	driveSet(66, 70, true);
	pidWait(Wait::CHAIN);
	// touch ladder or go to positive corner
	if(ladderBool) {
		swingSet(RIGHT_SWING, 225, 127, 60);
	} else {
		driveSet(48, 127);
		pidWait(Wait::CHAIN);
		turnSet(135, 100);
		pidWait(Wait::CHAIN);
		driveSet(24, 127);
	}
	pidWait(Wait::WAIT);
}

void red_7neg() {
	allianceColor = Colors::RED;
	setPosition(62.75, 18.5, 152);
	// score alliance and grab mogo
	driveSet(6.5, 90);
	setDunker(2000);
	pidWait(Wait::WAIT);
	driveSet(-38.6, 90);
	primeMogo();
	pidWaitUntil(-28.5_in);
	tareDunker();
	setMogo(true);
	setIntake(127);
	pidWait(Wait::CHAIN);
	// sweep ring rush rings
	moveToPoint({35, 57}, fwd, 70);
	pidWait(Wait::CHAIN);
	moveToPoint({14, 58}, fwd, 70);
	pidWait(Wait::WAIT);
	delayMillis(500);
	swingSet(RIGHT_SWING, -70, 127, 70, cw);
	pidWait(Wait::CHAIN);
	// grab bottom ring of ring stack, then score corner
	moveToPoint({23, 42}, fwd, 100);
	pidWait(Wait::CHAIN);
	moveToPoint({23, 22}, fwd, 100);
	pidWait(Wait::CHAIN);
	turnSet(225, 127);
	pidWait(Wait::CHAIN);
	// score corner
	driveSet(24, 127, true);
	pidWait(Wait::CHAIN);
	driveSet(-16, 127);
	pidWait(Wait::CHAIN);
	driveSet(24, 127, true);
	pidWait(Wait::CHAIN);
	driveSet(-12, 127);
	pidWait(Wait::CHAIN);
	// grab mid top ring
	turnSet(90, 100);
	pidWait(Wait::CHAIN);
	driveSet(66, 70, true);
	pidWait(Wait::CHAIN);
	// touch ladder or go to positive corner
	if(ladderBool) {
		swingSet(RIGHT_SWING, 225, 127, 60);
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

void red_4pos() {
	allianceColor = Colors::RED;
	setPosition(81.25, 18.5, 208);
	// score alliance and grab mogo
	driveSet(6.5, 90);
	setDunker(2000);
	pidWait(Wait::WAIT);
	driveSet(-38.6, 90);
	primeMogo();
	pidWaitUntil(-28.5_in);
	tareDunker();
	setMogo(true);
	setIntake(127);
	pidWait(Wait::CHAIN);
	// score bottom ring
	turnSet(100, 100);
	pidWait(Wait::CHAIN);
	driveSet(22, 100);
	// score corner
	pidWait(Wait::WAIT);
	turnSet(180, 100);
	pidWait(Wait::CHAIN);
	driveSet(24, 100);
	pidWait(Wait::CHAIN);
	turnSet(135, 100);
	driveSet(24, 127, true);
	pidWait(Wait::CHAIN);
	driveSet(-16, 127);
	pidWait(Wait::CHAIN);
	driveSet(24, 127, true);
	pidWait(Wait::CHAIN);
	driveSet(-12, 127);
	pidWait(Wait::CHAIN);
	// score mid top ring
	turnSet(-90, 100);
	pidWait(Wait::CHAIN);
	driveSet(66, 70, true);
	pidWait(Wait::CHAIN);
	// touch ladder or go to positive corner
	if(ladderBool) {
		swingSet(LEFT_SWING, 135, 127, 50, ccw);
	} else {
		driveSet(-12, 127);
    pidWait(Wait::CHAIN);
    turnSet(225, 100);
    pidWait(Wait::CHAIN);
    driveSet(-60, 127);
	}
	pidWait(Wait::WAIT);
}

void red_6pos() {
	allianceColor = Colors::RED;
	setPosition(81.25, 18.5, 208);
	// score alliance and grab mogo
	driveSet(6.5, 90);
	setDunker(2000);
	pidWait(Wait::WAIT);
	driveSet(-38.6, 90);
	primeMogo();
	pidWaitUntil(-28.5_in);
	tareDunker();
	setMogo(true);
	setIntake(127);
	pidWait(Wait::CHAIN);
	// grab middle rings with doinker
	moveToPoint({76, 58}, fwd, 90);
	pidWait(Wait::QUICK);
	setDoinker(true);
	// bring rings back and score all of them
	swingSet(RIGHT_SWING, 10, 60, 40, cw);
	pidWait(Wait::CHAIN);
	setDoinker(false);
	swingSet(LEFT_SWING, 90, 127, 80, cw);
	// score corner
	pidWait(Wait::WAIT);
	turnSet(180, 100);
	pidWait(Wait::CHAIN);
	driveSet(20, 100);
	pidWait(Wait::CHAIN);
	turnSet(135, 100);
	driveSet(24, 127, true);
	pidWait(Wait::CHAIN);
	driveSet(-16, 127);
	pidWait(Wait::CHAIN);
	driveSet(24, 127, true);
	pidWait(Wait::CHAIN);
	driveSet(-8, 127);
	pidWait(Wait::CHAIN);
	// score mid top ring
	turnSet(-90, 100);
	pidWait(Wait::CHAIN);
	driveSet(66, 70, true);
	pidWait(Wait::CHAIN);
	// touch ladder or go to positive corner
	if(ladderBool) {
		swingSet(LEFT_SWING, 135, 127, 50, ccw);
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
	allianceColor = Colors::BLUE;
	setPosition(81.25, 18.5, 208);
	// score alliance and grab mogo
	driveSet(6.5, 90);
	setDunker(2000);
	pidWait(Wait::WAIT);
	driveSet(-38.6, 90);
	primeMogo();
	pidWaitUntil(-28.5_in);
	tareDunker();
	setMogo(true);
	setIntake(127);
	pidWait(Wait::CHAIN);
	// sweep ring rush rings
	moveToPoint({109, 57}, fwd, 70);
	pidWait(Wait::CHAIN);
	moveToPoint({130, 58}, fwd, 70);
	pidWait(Wait::WAIT);
	delayMillis(500);
	swingSet(LEFT_SWING, 70, 127, 70, ccw);
	pidWait(Wait::CHAIN);
	// grab bottom ring of ring stack, and go to middle
	moveToPoint({121, 42}, fwd, 100);
	pidWait(Wait::CHAIN);
	moveToPoint({72, 24}, fwd, 100);
	pidWait(Wait::QUICK);
	setMogo(false);
	// grab mogo
	moveToPoint({48, 48}, rev, 90);
	delayMillis(500);
	primeMogo();
	pidWait(Wait::CHAIN);
	// grab bottom ring and middle ring
	turnSet(-90, 100);
	pidWait(Wait::CHAIN);
	driveSet(20, 100);
	pidWait(Wait::CHAIN);
	moveToPoint({68, 16}, fwd, 100);
	pidWait(Wait::CHAIN);
	// touch ladder
	if(ladderBool) {
		swingSet(RIGHT_SWING, 225, 127, 50, cw);
	} else {
		moveToPoint({16, 16}, fwd, 127);
		pidWait(Wait::CHAIN);
		turnSet(225, 100);
		pidWait(Wait::CHAIN);
		driveSet(24, 127);
	}
	pidWait(Wait::WAIT);
}

void blue_6neg() {
	allianceColor = Colors::BLUE;
	setPosition(81.25, 18.5, 208);
	// grab mogo
	driveSet(-32, 90);
	primeMogo();
	pidWaitUntil(-22.5_in);
	setMogo(true);
	setIntake(127);
	pidWait(Wait::CHAIN);
	// sweep ring rush rings
	moveToPoint({109, 57}, fwd, 70);
	pidWait(Wait::CHAIN);
	moveToPoint({130, 58}, fwd, 70);
	pidWait(Wait::WAIT);
	delayMillis(500);
	swingSet(LEFT_SWING, 70, 127, 70, ccw);
	pidWait(Wait::CHAIN);
	// grab bottom ring of ring stack, then score corner
	moveToPoint({121, 42}, fwd, 100);
	pidWait(Wait::CHAIN);
	moveToPoint({121, 22}, fwd, 100);
	pidWait(Wait::CHAIN);
	turnSet(135, 127);
	pidWait(Wait::CHAIN);
	// score corner
	driveSet(24, 127, true);
	pidWait(Wait::CHAIN);
	driveSet(-16, 127);
	pidWait(Wait::CHAIN);
	driveSet(24, 127, true);
	pidWait(Wait::CHAIN);
	driveSet(-12, 127);
	pidWait(Wait::CHAIN);
	// grab mid top ring
	turnSet(-90, 100);
	pidWait(Wait::CHAIN);
	driveSet(66, 70, true);
	pidWait(Wait::CHAIN);
	// touch ladder or go to positive corner
	if(ladderBool) {
		swingSet(LEFT_SWING, 135, 127, 60, ccw);
	} else {
		driveSet(48, 127);
		pidWait(Wait::CHAIN);
		turnSet(225, 100);
		pidWait(Wait::CHAIN);
		driveSet(24, 127);
	}
	pidWait(Wait::WAIT);
}

void blue_7neg() {
	allianceColor = Colors::BLUE;
	setPosition(81.25, 18.5, 208);
	// score alliance and grab mogo
	driveSet(6.5, 90);
	setDunker(2000);
	pidWait(Wait::WAIT);
	driveSet(-38.6, 90);
	primeMogo();
	pidWaitUntil(-28.5_in);
	tareDunker();
	setMogo(true);
	setIntake(127);
	pidWait(Wait::CHAIN);
	// sweep ring rush rings
	moveToPoint({109, 57}, fwd, 70);
	pidWait(Wait::CHAIN);
	moveToPoint({130, 58}, fwd, 70);
	pidWait(Wait::WAIT);
	delayMillis(500);
	swingSet(LEFT_SWING, 70, 127, 70, ccw);
	pidWait(Wait::CHAIN);
	// grab bottom ring of ring stack, then score corner
	moveToPoint({121, 42}, fwd, 100);
	pidWait(Wait::CHAIN);
	moveToPoint({121, 22}, fwd, 100);
	pidWait(Wait::CHAIN);
	turnSet(135, 127);
	pidWait(Wait::CHAIN);
	// score corner
	driveSet(24, 127, true);
	pidWait(Wait::CHAIN);
	driveSet(-16, 127);
	pidWait(Wait::CHAIN);
	driveSet(24, 127, true);
	pidWait(Wait::CHAIN);
	driveSet(-12, 127);
	pidWait(Wait::CHAIN);
	// grab mid top ring
	turnSet(-90, 100);
	pidWait(Wait::CHAIN);
	driveSet(66, 70, true);
	pidWait(Wait::CHAIN);
	// touch ladder or go to positive corner
	if(ladderBool) {
		swingSet(LEFT_SWING, 135, 127, 60, ccw);
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

void blue_4pos() {
	allianceColor = Colors::BLUE;
	setPosition(62.75, 18.5, 152);
	// score alliance and grab mogo
	driveSet(6.5, 90);
	setDunker(2000);
	pidWait(Wait::WAIT);
	driveSet(-38.6, 90);
	primeMogo();
	pidWaitUntil(-28.5_in);
	tareDunker();
	setMogo(true);
	setIntake(127);
	pidWait(Wait::CHAIN);
	// score bottom ring
	turnSet(-100, 100);
	pidWait(Wait::CHAIN);
	driveSet(22, 100);
	// score corner
	pidWait(Wait::WAIT);
	turnSet(180, 100);
	pidWait(Wait::CHAIN);
	driveSet(24, 100);
	pidWait(Wait::QUICK);
	turnSet(-135, 100);
	driveSet(24, 127, true);
	pidWait(Wait::CHAIN);
	driveSet(-16, 127);
	pidWait(Wait::CHAIN);
	driveSet(24, 127, true);
	pidWait(Wait::CHAIN);
	driveSet(-12, 127);
	pidWait(Wait::CHAIN);
	// score mid top ring
	turnSet(90, 100);
	pidWait(Wait::CHAIN);
	driveSet(66, 70, true);
	pidWait(Wait::CHAIN);
	// touch ladder or go to goal rush mogo
	if(ladderBool) {
		swingSet(RIGHT_SWING, 225, 127, 50);
	} else {
		driveSet(-12, 127);
    pidWait(Wait::CHAIN);
    turnSet(135, 100);
    pidWait(Wait::CHAIN);
    driveSet(-60, 127);
	}
	pidWait(Wait::WAIT);
}

void blue_6pos() {
	allianceColor = Colors::BLUE;
	setPosition(62.75, 18.5, 152);
	// score alliance and grab mogo
	driveSet(6.5, 90);
	setDunker(2000);
	pidWait(Wait::WAIT);
	driveSet(-38.6, 90);
	primeMogo();
	pidWaitUntil(-28.5_in);
	tareDunker();
	setMogo(true);
	setIntake(127);
	pidWait(Wait::CHAIN);
	// grab middle rings with doinker
	moveToPoint({68, 58}, fwd, 90);
	pidWait(Wait::QUICK);
	setDoinker(true);
	// bring rings back and score all of them
	swingSet(LEFT_SWING, -10, 60, 40, ccw);
	pidWait(Wait::CHAIN);
	setDoinker(false);
	swingSet(RIGHT_SWING, -90, 127, 80, ccw);
	// score corner
	pidWait(Wait::WAIT);
	turnSet(180, 100);
	pidWait(Wait::CHAIN);
	driveSet(20, 100);
	pidWait(Wait::CHAIN);
	turnSet(-135, 100);
	driveSet(24, 127, true);
	pidWait(Wait::CHAIN);
	driveSet(-16, 127);
	pidWait(Wait::CHAIN);
	driveSet(24, 127, true);
	pidWait(Wait::CHAIN);
	driveSet(-8, 127);
	pidWait(Wait::CHAIN);
	// score mid top ring
	turnSet(90, 100);
	pidWait(Wait::CHAIN);
	driveSet(66, 80, true);
	pidWait(Wait::CHAIN);
	if(ladderBool) {
		swingSet(RIGHT_SWING, 225, 127, 50);
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
	chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
	chassis.pid_wait();
}

void testexitconditions() {
  driveSet(-24, 80);
  primeMogo();
  pidWait(Wait::WAIT);
  setIntake(127);
  turnSet(90, 100);
  pidWait(Wait::WAIT);
  driveSet(24, 127);
  pidWait(Wait::WAIT);
  driveSet(-36, 127);
  pidWait(Wait::WAIT);
}

void red_testauto() {
	allianceColor = Colors::RED;
	setIntake(127);
	primeMogo();
}

void blue_testauto() {
	allianceColor = Colors::BLUE;
	setIntake(127);
	primeMogo();
}