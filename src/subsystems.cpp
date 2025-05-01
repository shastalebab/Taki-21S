#include "main.h"  // IWYU pragma: keep

// Values to determine dunker behavior
const int dunker_down_speed = 115;
const int dunker_up_speed = 127;
int dunker_current_max_speed = dunker_up_speed;

const vector<int> scoreStates = {1, 13};
const vector<int> descoreStates = {125, 130, 140, 145, 155, 170};

bool dunkerScoringState = true;
bool dunkerPreset = false;
bool usingDunkerTarget = true;

// Internal targets to aid tasks
Colors allianceColor = Colors::NEUTRAL;
bool mogoState = false;
int intakeTarget = 0;
int dunkerState = 0;

// Internal states to avoid tasks clashing
bool jamState = false;
bool ringDetected = false;
bool discarding = false;
bool resetting = false;
bool holding = false;
bool indexing = false;

//
// Wrappers
//

void setIntake(int speed) {
	if(autonMode != AutonMode::BRAIN) {
		intake.move(speed);
		intakeTarget = speed;
	}
}

void setIndexing() {
	indexing = true;
}

void setDunker(int position) {
	if(autonMode != AutonMode::BRAIN) {
		if(position > dunkerPID.target_get()) {
			dunker_current_max_speed = dunker_up_speed;
		} else {
			dunker_current_max_speed = dunker_down_speed;
		}
		dunkerPID.target_set(position);
		holding = false;
	}
}

double getDunker() {
	return dunkerSens.get_position() / 100.0;
}

void resetDunker() {
	if(autonMode != AutonMode::BRAIN) resetting = true;
}

void setMogo(bool state) {
	if(autonMode != AutonMode::BRAIN) mogomech.set(state);
}

void primeMogo() { mogoState = true; }

void setDoinker(bool state) {
	if(autonMode != AutonMode::BRAIN) doinker.set(state);
}

void setActuatedIntake(bool state) {
	if(autonMode != AutonMode::BRAIN) actuatedIntake.set(state);
}

//
// Operator Control
//

void setIntakeOp() {
	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
		if(!indexing) setIntake(127);
		setDunker(24);
		dunkerPreset = true;
		usingDunkerTarget = true;
		setIndexing();
	} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		setIntake(127);
	else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		setIntake(-127);
	else {
		setIntake(0);
		indexing = false;
	}
}

void setDunkerOp() {
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
		vector<int> dunkerStates = dunkerScoringState ? scoreStates : descoreStates;

		dunkerState++;
		dunkerState %= dunkerStates.size();
		dunkerPreset = true;
		usingDunkerTarget = true;
		if(getDunker() > 30 && dunkerScoringState) dunkerState = 1;
		if(dunkerState == 0 && dunkerScoringState)
			resetDunker();
		else
			setDunker(dunkerStates[dunkerState]);
	} else {
		dunkerScoringState = true;
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			usingDunkerTarget = true;
			dunkerPreset = true;
			dunkerScoringState = false;
			setDunker(descoreStates[dunkerState]);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			dunker.move(127);
			setDunker(getDunker());
			usingDunkerTarget = false;
			dunkerPreset = false;
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			dunker.move(-127);
			setDunker(getDunker());
			usingDunkerTarget = false;
			dunkerPreset = false;
		} else if(!dunkerPreset and resetting == false) {
			usingDunkerTarget = true;
			dunker.move(0);
		}
	}
	if(dunkerPID.target_get() > 270)
		setDunker(270);
	else if(dunkerPID.target_get() < 0)
		setDunker(0);
}

void setMogoOp() { mogomech.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)); }

void setDoinkerOp() { setDoinker(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)); }

//
// Color sort
//

void discard() {
	intake.move(-127);
	pros::delay(70);
	setIntake(intakeTarget);
	discarding = false;
	ringDetected = false;
}

void colorSet(Colors color) {
	// Set on screen elements to the corresponding color
	lv_color32_t color_use = theme_accent;
	if(color == Colors::RED)
		color_use = red;
	else if(color == Colors::BLUE)
		color_use = blue;
	lv_obj_set_style_outline_color(colorind, color_use, LV_PART_MAIN);
}

Colors colorGet() {
	auto hue = colorSens.get_hue();
	if(colorSens.get_proximity() > 100) {
		if((hue > 340 && hue < 360) || (hue > 0 && hue < 15))
			return Colors::RED;
		else if(hue > 200 && hue < 225)
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
	ringSens.set_integration_time(10);
	colorSens.set_integration_time(10);
	colorSens.set_led_pwm(100);
	while(true) {
		color = colorGet();
		colorSet(color);
		if(!jamState && !pros::competition::is_disabled()) {
			cout << ringSens.get_proximity() << "\n";
			if(colorCompare(color) && !discarding) {
				discarding = true;
			} else if(discarding) {
				if(ringSens.get_proximity() == 255 && util::sgn(intake.get_actual_velocity()) == 1) ringDetected = true;
				if(ringDetected && ringSens.get_proximity() < 130) discard();
			} else if(allianceColor == color && allianceColor != Colors::NEUTRAL && indexing) {
				setIntake(0);
				indexing = false;
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
		if(pros::competition::is_autonomous() && mogoState == true) {
			if(distanceSens.get() < 45) {
				setMogo(true);
				mogoState = false;
			}
		}
		pros::delay(10);
	}
}

void dunkerTask() {
	int resetTime = 0;
	while(true) {
		if(resetting) {
			dunker.move(-127);
			if(abs(dunker.get_actual_velocity()) < 5) resetTime++;
			if(resetTime > 10) {
				resetTime = 0;
				holding = true;
				resetting = false;
			}
		} else if(holding) {
			dunker.move(-3);
		} else if(usingDunkerTarget) {
			double output = dunkerPID.compute(getDunker());
			output = ez::util::clamp(output, dunker_current_max_speed);
			dunker.move(output);
		}
		pros::delay(10);
	}
}

void unjamTask() {
	int jamtime = 0;
	while(true) {
		if(intake.get_temperature() < 50) {
			if(dunkerState != 1 && dunkerScoringState == true) {
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