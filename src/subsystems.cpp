#include "main.h"  // IWYU pragma: keep


// Values to determine dunker behavior
const int dunker_down_speed = 115;
const int dunker_up_speed = 127;
int dunker_current_max_speed = dunker_up_speed;

const vector<int> scoreStates = {1, 12};
const vector<int> descoreStates = {159, 171, 181, 194, 205, 217};

bool dunkerScoringState = true;
bool dunkerPreset = false;
bool usingDunkerTarget = true;

// Internal targets to aid tasks
Colors allianceColor = Colors::NEUTRAL;
Colors matchColor = allianceColor;
bool mogoState = false;
int intakeTarget = 0;
int dunkerState = 0;

// Internal states to avoid tasks clashing
bool jamState = false;
bool ringDetected = false;
bool jamDisabled = false;
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

void setIndexing() { indexing = true; }

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

void setDunker(int position, int max_speed) {
	if(autonMode != AutonMode::BRAIN) {
		dunker_current_max_speed = max_speed;
		dunkerPID.target_set(position);
		holding = false;
	}
}

double getDunker() { return dunkerSens.get_position() / 100.0; }

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
	if(autonMode != AutonMode::BRAIN) {
		actuatedIntake.set(state);
		antiJamDisabled(state);
	}
}

void antiJamDisabled(bool state) {
	jamDisabled = state;
}

void setAlliance(Colors alliance) {
	allianceColor = alliance;
	matchColor = alliance;
}
void sendHaptic(string input) { controllerInput = input; }

//
// Operator Control
//

void setIntakeOp() {
	if(!discarding && !jamState) {
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			colorToggle();
			sendHaptic(".");
			pros::delay(50);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			if(!indexing) setIntake(90);
			setDunker(25);
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
	} else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
		resetDunker();
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
		resetDunker();
}

void setMogoOp() { mogomech.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)); }

void setDoinkerOp() { setDoinker(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)); }

//
// Color sort
//

void colorToggle() {
	if(allianceColor == matchColor)
		allianceColor = Colors::NEUTRAL;
	else
		allianceColor = matchColor;
}

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
	double hue = 0;
	if(ringSens.get_proximity() > 100) {
		hue = colorSens.get_hue();
		if((hue > 340 && hue < 360) || (hue > 0 && hue < 20))
			return Colors::RED;
		else if(hue > 210 && hue < 240)
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
	int sortTime = 0;
	colorSens.set_integration_time(10);
	ringSens.set_integration_time(10);
	colorSens.set_led_pwm(100);
	while(true) {
		color = colorGet();
		colorSet(color);
		if(!jamState && !pros::competition::is_disabled()) {
			if(colorCompare(color) && !discarding) {
				discarding = true;
			} else if(discarding) {
				sortTime++;
				if(ringSens.get_proximity() > 220 && util::sgn(intake.get_actual_velocity()) == 1) ringDetected = true;
				if(ringDetected && ringSens.get_proximity() < 130) {
					discard();
					sortTime = 0;
				} else if(sortTime > 50) {
					discarding = false;
					sortTime = 0;
				}
			} else if(indexing) {
				if((matchColor == color) && matchColor != Colors::NEUTRAL) {
					setIntake(0);
					indexing = false;
				}
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
			if(dunkerState != 1 || !dunkerScoringState) {
				if(!jamState && !jamDisabled && intakeTarget != 0 && abs(intake.get_actual_velocity()) <= 20) {
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