#include "main.h"
#include "EZ-Template/util.hpp"
#include "pros/misc.h"
#include "subsystems.hpp"

// Chassis constructor
ez::Drive chassis(
	// These are your drive motors, the first motor is used for sensing!
	{-4, -5, -6},  // Left Chassis Ports (negative port will reverse it!)
	{1, 2, 3},	   // Right Chassis Ports (negative port will reverse it!)

	21,				// IMU Port
	2.8469137379,	// Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
	450);			// Wheel RPM = cartridge * (motor gear / wheel gear)

void initialize() {
	// Print our branding over your terminal :D
	ez::ez_template_print();

	pros::delay(500);  // Stop the user from doing anything while legacy ports configure

	// Configure your chassis controls
	chassis.opcontrol_curve_buttons_toggle(false);	// Enables modifying the controller curve with buttons on the joysticks
	chassis.opcontrol_drive_activebrake_set(2.0);	// Sets the active brake kP. We recommend ~2.  0 will disable.
	chassis.opcontrol_curve_default_set(
		3.0, 0.0);	// Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

	// Set the drive to your own constants from autons.cpp!
	default_constants();

	// Autonomous Selector using LLEMU
	auton_sel.selector_populate({{red_4pos, "red_4pos", "red 4 + 1 pos", red},
								 {red_6pos, "red_6pos", "red 6 + 1 pos", red},
								 {red_solowp, "red_solowp", "red negative solo win point", red},
								 {red_6neg, "red_6neg", "red 6 ring neg", red},
								 {red_7neg, "red_7neg", "red 6 + 1 ring neg", red},
								 {blue_4pos, "blue_4pos", "blue 4 + 1 pos", blue},
								 {blue_6pos, "blue_6pos", "blue 6 + 1 pos", blue},
								 {blue_solowp, "blue_solowp", "blue negative solo win point", blue},
								 {blue_6neg, "blue_6neg", "blue 6 ring neg", blue},
								 {blue_7neg, "blue_7neg", "blue 6 + 1 ring neg", blue},
								 {red_testauto, "red_testauto", "test color sort red", red},
								 {blue_testauto, "blue_testauto", "test color sort blue", blue},
								 {move_forward, "move_forward", "move forward 24 inches", gray},
								{testexitconditions, "test exit", "test exit conditions", lv_color_hex(0x00FF00)}});

	// Initialize chassis and auton selector
	chassis.initialize();
	uiInit();
	pros::Task ColorTask(colorTask);
	pros::Task MogoTask(mogoTask);
	pros::Task DunkerTask(dunkerTask);
	pros::Task UnjamTask(unjamTask);
	pros::Task PathViewerTask(pathViewerTask);
	master.rumble(chassis.drive_imu_calibrated() ? "." : "---");

	dunker.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {
	// . . .
}

void competition_initialize() { 
	autonMode = AutonMode::BRAIN;
	bool checked = false;
	while(true) {
		if(hookSens.get_value() < 2800) checked = true;
		if(checked == true && hookSens.get_value() > 2800) {
			getPos();
			checked = false;
		}

		pros::delay(util::DELAY_TIME);
	}
 }

void autonomous() {
	chassis.pid_targets_reset();				// Resets PID targets to 0
	chassis.drive_imu_reset();					// Reset gyro position to 0
	chassis.drive_sensor_reset();				// Reset drive sensors to 0
	chassis.odom_xyt_set(0_in, 0_in, 0_deg);	// Set the current position, you can start at a specific position with this
	chassis.drive_brake_set(MOTOR_BRAKE_HOLD);	// Set motors to hold.  This helps autonomous consistency

	autonMode = AutonMode::PLAIN;
	autonPath = {};
	if(delayBool) delayMillis(1000);
	auton_sel.selector_callback();	// Calls selected auton from autonomous selector
}

void opcontrol() {
	chassis.drive_brake_set(pros::E_MOTOR_BRAKE_BRAKE);
	setMogo(false);
	tareDunker();
	autonMode = AutonMode::DRIVER;

	while(true) {
		chassis.opcontrol_tank();  // Tank control

		setIntakeOp();
		setDunkerOp();
		setMogoOp();
		setDoinkerOp();

		pros::delay(ez::util::DELAY_TIME);	// This is used for timer calculations!  Keep this ez::util::DELAY_TIME
	}
}
