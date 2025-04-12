#include "main.h"  // IWYU pragma: keep

// Values to determine dunker behavior
bool dunkerPreset = false;
bool usingDunkerTarget = true;

// Internal targets to aid tasks
Colors allianceColor = Colors::NEUTRAL;
AutoMogo mogoState = AutoMogo::OFF;
int intakeTarget = 0;
bool dunkerState = 0;

// Internal states to avoid tasks clashing
bool jamState = false;
bool discarding = false;

//
// Wrappers
//

void setIntake(int speed) {
	intake.move(speed);
	intakeTarget = speed;
}

void setDunker(int position) { dunkerPID.target_set(position); }

void setMogo(bool state) { mogomech.set(state); }

//
// Operator Control
//

void setIntakeOp() {
	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		setIntake(127);
	} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
		setIntake(-127);
	} else
		setIntake(0);
}

void setDunkerOp() {
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
		dunkerState = !dunkerState;
		dunkerPreset = true;
		if(dunker.get_position() > 300) dunkerState = false;
		if(dunkerState)
			setDunker(245);
		else
			setDunker(10);
	} else {
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			dunker.move(127);
			setDunker(dunker.get_position());
			usingDunkerTarget = false;
			dunkerPreset = false;
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			dunker.move(-127);
			setDunker(dunker.get_position());
			usingDunkerTarget = false;
			dunkerPreset = false;
		} else if(!dunkerPreset) {
			usingDunkerTarget = true;
		}
	}
	if(dunkerPID.target_get() > 3000)
		setDunker(3000);
	else if(dunkerPID.target_get() < 10)
		setDunker(10);
}

void setMogoOp() { mogomech.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)); }

//
// Color sort
//

void discard() {
	intake.move(-127);
	pros::delay(80);
	setIntake(intakeTarget);
	discarding = false;
}

void colorSet(Colors color) {
	// Set on screen elements to the corresponding color
}

Colors colorGet() {
	auto hue = colorSens.get_hue();
	if(colorSens.get_proximity() > 100) {
		if((hue > 340 && hue < 360) || (hue > 0 && hue < 15))
			return Colors::RED;
		else if(hue > 210 && hue < 225)
			return Colors::BLUE;
	}
	return Colors::NEUTRAL;
}

bool colorCompare(Colors color) {
	if((int)allianceColor < 2 && (int)color < 2) return allianceColor != color;
	return false;
}

void colorTask() {
	Colors color;
	colorSens.set_integration_time(10);
	colorSens.set_led_pwm(100);
	while(true) {
		color = colorGet();
		colorSet(color);
		if(!jamState && pros::competition::is_autonomous() && !pros::competition::is_disabled()) {
			if(colorCompare(color) && !discarding) {
				discarding = true;
			} else if(discarding) {
				if(hookSens.get_value() < 2800 && util::sgn(intake.get_actual_velocity()) == 1) discard();
			}
		}
		pros::delay(10);
	}
}

//
// Other tasks
//

void mogoTask() {
    while(true) {
		if(pros::competition::is_autonomous() && mogoState == AutoMogo::PRIMED) {
			if(distanceSens.get() < 40) {
				mogomech.set(true);
				mogoState = AutoMogo::OFF;
			}
		}
		pros::delay(10);
	}
}

void dunkerTask() {
	int taretime = 0;
	while(true) {
		if(usingDunkerTarget) dunker.move(dunkerPID.compute(dunker.get_position()));
		pros::delay(10);
	}
}

void unjamTask() {
	int jamtime = 0;
	while(true) {
		if(intake.get_temperature() < 50) {
			if(!dunkerState) {
				if(!jamState && intakeTarget != 0 && abs(intake.get_actual_velocity()) <= 20) {
					jamtime++;
					if(jamtime > 20) {
						jamtime = 0;
						jamState = true;
					}
				}

				if(jamState) {
					intake.move(-intakeTarget);
					jamtime++;
					if(jamtime > 20) {
						jamtime = 0;
						jamState = false;
						setIntake(intakeTarget);
					}
				}
			}
		}
		pros::delay(10);
	}
}