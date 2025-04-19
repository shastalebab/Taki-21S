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
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 25.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.25, 0.05, 25.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

	// Exit conditions
	chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
	chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
	chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
	chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
	chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
	chassis.pid_turn_chain_constant_set(3_deg);
	chassis.pid_swing_chain_constant_set(5_deg);
	chassis.pid_drive_chain_constant_set(3_in);

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

	chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void testauto() {
  allianceColor = Colors::RED;
  setIntake(127);
  setMogo(true);
  pros::delay(1000000);
}

//
// RED NEGATIVE
//

void red_6neg() {
	allianceColor = Colors::RED;
	setPosition(48, 20, 180);
	// grab mogo
	moveToPoint({48, 44}, rev, 80);
	pidWait(Wait::CHAIN);
  setMogo(true);
	setIntake(127);
	// sweep ring rush rings
	pidWait(Wait::QUICK);
	moveToPoint({38, 58}, fwd, 70);
  pidWait(Wait::CHAIN);
	moveToPoint({14, 59}, fwd, 70);
	pidWait(Wait::WAIT);
  delayMillis(500);
	swingSet(ez::RIGHT_SWING, -75, 127, 70, ez::cw);
	pidWait(Wait::CHAIN);
	// grab bottom ring of ring stack, then score corner
	moveToPoint({23, 42}, fwd, 100);
	pidWait(Wait::CHAIN);
	moveToPoint({23, 24}, fwd, 100);
	pidWait(Wait::CHAIN);
	turnSet(225, 127);
	pidWait(Wait::CHAIN);
  // score corner
	driveSet(24, 90);
  pidWait(Wait::WAIT);
	driveSet(-16, 80);
	pidWait(Wait::CHAIN);
	// grab mid top ring
	turnSet(90, 100);
  pidWait(Wait::CHAIN);
  driveSet(66, 100, true);
  pidWaitUntil(30);
  chassis.pid_speed_max_set(30);
	pidWait(Wait::CHAIN);
  // touch ladder
	swingSet(ez::RIGHT_SWING, 225, 127, 60);
  pidWait(Wait::WAIT);
}

//
// RED POSITIVE
//

void red_5pos() {
  allianceColor = Colors::BLUE;
  setPosition(83.5, 8, 238.88);
	// score alliance and grab mogo
  setDunker(2000);
  delayMillis(500);
	swingSet(ez::LEFT_SWING, 180, 90, 32, ez::ccw);
	pidWait(Wait::CHAIN);
  tareDunker();
  driveSet(-30, 80);
  pidWaitUntil(-24);
  setMogo(true);
	setIntake(127);
  pidWait(Wait::CHAIN);
  setPosition(93.25, 54);
  // grab middle rings with doinker
  moveToPoint({79, 62}, fwd, 90);
  pidWait(Wait::QUICK);
  setDoinker(true);
  // bring rings back and score all of them
  swingSet(ez::RIGHT_SWING, 0, 60, 40, ez::cw);
  pidWait(Wait::CHAIN);
  driveSet(14, 100);
  pidWait(Wait::CHAIN);
  turnSet(90, 100);
  pidWait(Wait::CHAIN);
  driveSet(20, 100);
  // score corner
  pidWait(Wait::WAIT);
  turnSet(180, 100);
  pidWait(Wait::CHAIN);
  driveSet(15, 100);
  pidWait(Wait::CHAIN);
  turnSet(135, 100);
  driveSet(24, 127);
  pidWait(Wait::WAIT);
	driveSet(-16, 127);
	pidWait(Wait::CHAIN);
  driveSet(24, 127);
  pidWait(Wait::WAIT);
  driveSet(-8, 127);
  pidWait(Wait::CHAIN);
  turnSet(-90, 100);
  pidWait(Wait::CHAIN);
  driveSet(66, 80, true);
	pidWait(Wait::CHAIN);
  // touch ladder
	swingSet(ez::LEFT_SWING, 135, 127, 50);
  pidWait(Wait::WAIT);
}

//
// BLUE NEGATIVE
//

void blue_6neg() {
  allianceColor = Colors::BLUE;
	setPosition(96, 20, 180);
	// grab mogo
	moveToPoint({96, 44}, rev, 80);
	pidWait(Wait::CHAIN);
  setMogo(true);
	setIntake(127);
	// sweep ring rush rings
	pidWait(Wait::QUICK);
	moveToPoint({106, 58}, fwd, 70);
  pidWait(Wait::CHAIN);
	moveToPoint({130, 59}, fwd, 70);
	pidWait(Wait::WAIT);
  delayMillis(500);
	swingSet(ez::LEFT_SWING, 75, 127, 70, ez::ccw);
	pidWait(Wait::CHAIN);
	// grab bottom ring of ring stack, then score corner
	moveToPoint({121, 42}, fwd, 100);
	pidWait(Wait::CHAIN);
	moveToPoint({121, 24}, fwd, 100);
	pidWait(Wait::CHAIN);
	turnSet(135, 127);
	pidWait(Wait::CHAIN);
  // score corner
	driveSet(24, 90);
  pidWait(Wait::WAIT);
	driveSet(-16, 80);
	pidWait(Wait::CHAIN);
	// grab mid top ring
	turnSet(-90, 100);
  pidWait(Wait::CHAIN);
  driveSet(66, 100, true);
  pidWaitUntil(30);
  chassis.pid_speed_max_set(30);
	pidWait(Wait::CHAIN);
  // touch ladder
	swingSet(ez::LEFT_SWING, 135, 127, 60);
  pidWait(Wait::WAIT);
}

//
// BLUE POSITIVE
//

void blue_5pos() {
  allianceColor = Colors::BLUE;
  setPosition(60.5, 8, 121.12);
	// score alliance and grab mogo
  setDunker(2000);
  delayMillis(500);
	swingSet(ez::RIGHT_SWING, 180, 90, 32, ez::cw);
	pidWait(Wait::CHAIN);
  tareDunker();
  driveSet(-30, 80);
  pidWaitUntil(-24);
  setMogo(true);
	setIntake(127);
  pidWait(Wait::CHAIN);
  setPosition(50.75, 54);
  // grab middle rings with doinker
  moveToPoint({65, 62}, fwd, 90);
  pidWait(Wait::QUICK);
  setDoinker(true);
  // bring rings back and score all of them
  swingSet(ez::LEFT_SWING, 0, 60, 40, ez::ccw);
  pidWait(Wait::CHAIN);
  driveSet(14, 100);
  pidWait(Wait::CHAIN);
  turnSet(-90, 100);
  pidWait(Wait::CHAIN);
  driveSet(20, 100);
  // score corner
  pidWait(Wait::WAIT);
  turnSet(180, 100);
  pidWait(Wait::CHAIN);
  driveSet(15, 100);
  pidWait(Wait::CHAIN);
  turnSet(-135, 100);
  driveSet(24, 127);
  pidWait(Wait::WAIT);
	driveSet(-16, 127);
	pidWait(Wait::CHAIN);
  driveSet(24, 127);
  pidWait(Wait::WAIT);
  driveSet(-8, 127);
  pidWait(Wait::CHAIN);
  turnSet(90, 100);
  pidWait(Wait::CHAIN);
  driveSet(66, 80, true);
	pidWait(Wait::CHAIN);
  // touch ladder
	swingSet(ez::RIGHT_SWING, 225, 127, 50);
  pidWait(Wait::WAIT);
}

///
// Drive Example
///
void drive_example() {
	// The first parameter is target inches
	// The second parameter is max speed the robot will drive at
	// The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
	// for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Turn Example
///

void three_ring() {
  //grab mogo
  chassis.pid_drive_set(-26_in,80, true);
    chassis.pid_wait_until(-21);
    setMogo(true);
    chassis.pid_wait();
  //turn grab other ring
  setIntake(127);
  chassis.pid_wait();
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(10000);
}
