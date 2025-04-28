#include "main.h"  // IWYU pragma: keep

// Values to determine dunker behavior
const int dunker_down_speed = 100;
const int dunker_up_speed = 127;
int dunker_current_max_speed = dunker_up_speed;

const vector<int> scoreStates = {10, 130};
const vector<int> descoreStates = {1250, 1300, 1400, 1450, 1550, 1700};

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
bool taring = false;
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
	}
}

void tareDunker() {
	if(autonMode != AutonMode::BRAIN) taring = true;
}

void setMogo(bool state) {
	if(autonMode != AutonMode::BRAIN) mogomech.set(state);
}

void primeMogo() { mogoState = true; }

void setDoinker(bool state) {
	if(autonMode != AutonMode::BRAIN) doinker.set(state);
}

bool setreset = false;

void getPos() {
	setreset = !setreset;
	if(setreset) chassis.drive_sensor_reset();
	else cout << "Left: " << util::to_string_with_precision(chassis.drive_sensor_left()) <<
	"\nRight: " << util::to_string_with_precision(chassis.drive_sensor_right()) << "\n";
}

//
// Operator Control
//

void setIntakeOp() {
	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
		if(!indexing) setIntake(127);
		setDunker(320);
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
		if(dunker.get_position() > 300 && dunkerScoringState) dunkerState = 1;
		if(dunkerState == 0 && dunkerScoringState)
			tareDunker();
		else
			setDunker(dunkerStates[dunkerState]);
	} else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
		tareDunker();
	} else {
		dunkerScoringState = true;
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			usingDunkerTarget = true;
			dunkerPreset = true;
			dunkerScoringState = false;
			setDunker(descoreStates[dunkerState]);
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			dunker.move(127);
			setDunker(dunker.get_position());
			usingDunkerTarget = false;
			dunkerPreset = false;
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			dunker.move(-127);
			setDunker(dunker.get_position());
			usingDunkerTarget = false;
			dunkerPreset = false;
		} else if(!dunkerPreset and taring == false) {
			usingDunkerTarget = true;
			dunker.move(0);
		}
	}
	if(dunkerPID.target_get() > 3000)
		setDunker(3000);
	else if(dunkerPID.target_get() < 10)
		setDunker(10);
}

void setMogoOp() { mogomech.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)); }

void setDoinkerOp() { setDoinker(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)); }

//
// Color sort
//

void discard() {
	intake.move(-127);
	pros::delay(80);
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
	colorSens.set_integration_time(10);
	colorSens.set_led_pwm(100);
	while(true) {
		color = colorGet();
		colorSet(color);
		if(!jamState && !pros::competition::is_disabled()) {
			if(colorCompare(color) && !discarding) {
				discarding = true;
			} else if(discarding) {
				if(hookSens.get_value() < 2800 && util::sgn(intake.get_actual_velocity()) == 1) ringDetected = true;
				if(ringDetected && hookSens.get_value() > 2800) discard();
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
	int taretime = 0;
	while(true) {
		if(taring) {
			dunker.move(-127);
			if(abs(dunker.get_actual_velocity()) < 5) taretime++;
			if(taretime > 10) {
				dunker.move(0);
				pros::delay(10);
				dunker.set_zero_position(-60);
				setDunker(10);
				taretime = 0;
				taring = false;
			}
		} else if(usingDunkerTarget) {
			double output = dunkerPID.compute(dunker.get_position());
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