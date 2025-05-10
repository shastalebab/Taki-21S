#include "main.h"

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
	auton_sel.selector_populate({{red_goalrush, "red_goalrush", "red 2 + 1 + 1 + 1 pos", red},
								 {red_5pos, "red_5pos", "red 5 + 1 pos", red},
								 {red_solowp, "red_solowp", "red negative solo win point", red},
								 {red_7neg, "red_7neg", "red 5 + 1 + 1 ring neg", red},
								 {red_7greed, "red_7greed", "red 6 + 1 ring neg", red},
								 {blue_goalrush, "blue_goalrush", "blue 2 + 1 + 1 + 1 pos", blue},
								 {blue_5pos, "blue_5pos", "blue 5 + 1 pos", blue},
								 {blue_solowp, "blue_solowp", "blue negative solo win point", blue},
								 {blue_7neg, "blue_7neg", "blue 5 + 1 + 1 ring neg", blue},
								 {blue_7greed, "blue_7greed", "blue 6 + 1 ring neg", blue},
								 {red_testauto, "red_testauto", "test color sort red", red},
								 {blue_testauto, "blue_testauto", "test color sort blue", blue},
								 {move_forward, "move_forward", "move forward 24 inches", gray},
								{tuning, "testing", "test stuff out", lv_color_hex(0x52f160)}});

	// Initialize chassis and auton selector
	chassis.initialize();
	dunker.tare_position();
	uiInit();
	pros::Task ColorTask(colorTask);
	pros::Task MogoTask(mogoTask);
	pros::Task DunkerTask(dunkerTask);
	pros::Task UnjamTask(unjamTask);
	pros::Task ControllerTask(controllerTask);
	pros::Task PathViewerTask(pathViewerTask);
	pros::Task AngleCheckTask(angleCheckTask);
	master.rumble(chassis.drive_imu_calibrated() ? "." : "---");

	dunker.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {
	// . . .
}

void competition_initialize() { 
	autonMode = AutonMode::BRAIN;
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
	antiJamDisabled(false);
	auton_sel.selector_callback();	// Calls selected auton from autonomous selector
}

void opcontrol() {
	chassis.drive_brake_set(pros::E_MOTOR_BRAKE_BRAKE);
	autonMode = AutonMode::DRIVER;
	setActuatedIntake(false);

	while(true) {
		chassis.opcontrol_tank();  // Tank control

		setIntakeOp();
		setDunkerOp();
		setMogoOp();
		setDoinkerOp();

		pros::delay(ez::util::DELAY_TIME);	// This is used for timer calculations!  Keep this ez::util::DELAY_TIME
	}
}
