#include "main.h"  // IWYU pragma: keep

// // // // // // Tasks & Non-UI // // // // // //

//
// Object creation
//

lv_obj_t* main_tv = lv_tileview_create(NULL);
lv_obj_t* colorind = lv_obj_create(main_tv);
lv_obj_t* autoSelector = lv_tileview_add_tile(main_tv, 0, 1, LV_DIR_NONE);
lv_obj_t* motorTemps = lv_tileview_add_tile(main_tv, 0, 0, LV_DIR_NONE);
lv_obj_t* pathPlanning = lv_tileview_add_tile(main_tv, 0, 2, LV_DIR_NONE);
lv_obj_t* pageUp[2] = {lv_label_create(main_tv), lv_label_create(main_tv)};
lv_obj_t* pageDown[2] = {lv_label_create(main_tv), lv_label_create(main_tv)};

lv_obj_t* autonTable = lv_list_create(autoSelector);
lv_obj_t* autonUp = lv_label_create(autoSelector);
lv_obj_t* autonDown = lv_label_create(autoSelector);
lv_obj_t* autonDesc = lv_label_create(autoSelector);
lv_obj_t* autonOptions = lv_img_create(autoSelector);
lv_obj_t* autonOptionsOutline = lv_img_create(autoSelector);
lv_obj_t* autonDelay = lv_obj_create(autoSelector);
lv_obj_t* autonLadder = lv_obj_create(autoSelector);
lv_obj_t* autonField = lv_img_create(autoSelector);
lv_obj_t* autonRobot = lv_img_create(autonField);
lv_obj_t* angleViewer;
lv_obj_t* angleText;

lv_obj_t* motorBoxes[8] = {lv_label_create(motorTemps), lv_label_create(motorTemps), lv_label_create(motorTemps), lv_label_create(motorTemps),
						   lv_label_create(motorTemps), lv_label_create(motorTemps), lv_label_create(motorTemps), lv_label_create(motorTemps)};

lv_obj_t* plannerField = lv_img_create(pathPlanning);
lv_obj_t* plannerRobot = lv_img_create(pathPlanning);
lv_obj_t* plannerDegrees = lv_arc_create(pathPlanning);
lv_obj_t* plannerDegreesInd = lv_label_create(pathPlanning);
lv_obj_t* plannerValueUp = lv_btn_create(pathPlanning);
lv_obj_t* plannerValueDown = lv_btn_create(pathPlanning);
lv_obj_t* plannerTimeline = lv_slider_create(pathPlanning);
lv_obj_t* plannerLabels[9] = {lv_label_create(pathPlanning), lv_label_create(motorTemps), lv_label_create(motorTemps),
							  lv_label_create(motorTemps),	 lv_label_create(motorTemps), lv_label_create(motorTemps),
							  lv_label_create(motorTemps),	 lv_label_create(motorTemps), lv_label_create(motorTemps)};

lv_style_t taki;

LV_IMG_DECLARE(red_alliance);
LV_IMG_DECLARE(blue_alliance);
LV_IMG_DECLARE(auton_options);
LV_IMG_DECLARE(auton_options_outline);
LV_IMG_DECLARE(robot);

// // // // // // Tasks & Non-UI // // // // // //

//
// Auton Selector
//

bool delayBool = false;
bool ladderBool = true;
bool aligning = false;

AutonSel auton_sel;

void AutonSel::selector_populate(vector<AutonObj> auton_list) { autons.insert(autons.end(), auton_list.begin(), auton_list.end()); }

void angleCheckTask() {
	while(true) {
		if(aligning) {
			auto target = autonPath.size() > 0 ? autonPath[0].t : 0;
			lv_label_set_text(angleText, (util::to_string_with_precision(chassis.drive_imu_get(), 2) + " °" + "\ntarget: " + util::to_string_with_precision(target, 2)).c_str());
			if(target + 0.15 >= chassis.drive_imu_get() && target - 0.15 <= chassis.drive_imu_get()) 
				lv_obj_set_style_bg_color(angleViewer, green, LV_PART_MAIN);
			else lv_obj_set_style_bg_color(angleViewer, red, LV_PART_MAIN);
		}
	pros::delay(10);
	}
}

//
// Controller Updates
//

void controllerTask() {
	string pattern = "";
	int timer = 0;
	float tempDrive;
	float tempIntake;
	float tempDunker;
	while(true) {
		// Update timer and rumble controller
		if(!pros::competition::is_autonomous() && !pros::competition::is_disabled()) {
			if(timer == 475) 
				pattern = "-";
			else if(timer == 375)
				pattern = "--";
			else if((timer >= 350 && timer < 375) || (timer >= 500 && timer < 525))
				pattern = ".";
			else
				pattern = "";
			if(timer % 5 == 0) master.rumble(pattern.c_str());
			timer++;
		}
		pros::delay(50);

		// Update temperature variables and print to controller
		tempDrive = (chassis.left_motors[0].get_temperature() + chassis.left_motors[1].get_temperature() + chassis.left_motors[2].get_temperature() +
					 chassis.right_motors[0].get_temperature() + chassis.right_motors[1].get_temperature() + chassis.right_motors[2].get_temperature()) /
					6;
		tempIntake = intake.get_temperature();
		tempDunker = dunker.get_temperature();

		if(tempDrive <= 30)
			pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "drive: cool, %.0f°C", tempDrive);
		else if(tempDrive > 30 && tempDrive <= 50)
			pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "drive: warm, %.0f°C", tempDrive);
		else if(tempDrive > 50)
			pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "drive: hot, %.0f°C", tempDrive);
		pros::delay(50);

		if(tempIntake <= 30)
			pros::c::controller_print(pros::E_CONTROLLER_MASTER, 1, 0, "intke: cool, %.0f°C", tempIntake);
		else if(tempIntake > 30 && tempIntake <= 50)
			pros::c::controller_print(pros::E_CONTROLLER_MASTER, 1, 0, "intke: warm, %.0f°C", tempIntake);
		else if(tempIntake > 50)
			pros::c::controller_print(pros::E_CONTROLLER_MASTER, 1, 0, "intke: hot, %.0f°C", tempIntake);
		pros::delay(50);

		if(tempDunker <= 30)
			pros::c::controller_print(pros::E_CONTROLLER_MASTER, 2, 0, "dunkr: cool, %.0f°C", tempDunker);
		else if(tempDunker > 30 && tempDunker <= 50)
			pros::c::controller_print(pros::E_CONTROLLER_MASTER, 2, 0, "dunkr: warm, %.0f°C", tempDunker);
		else if(tempDunker > 50)
			pros::c::controller_print(pros::E_CONTROLLER_MASTER, 2, 0, "dunkr: hot, %.0f°C", tempDunker);
		pros::delay(50);
	}
}

//
// Path Planning
//

int pathIter = 0;
vector<Coordinate> pathDisplay;

void resetViewer(bool full) {
	if(full) {
		auto preference = autonMode;
		autonMode = AutonMode::BRAIN;
		autonPath = {};
		auton_sel.selector_callback();
		pathDisplay = smoothPath(injectPath(autonPath, 1), 1, 4);
		autonMode = preference;
		lv_img_set_src(autonField, &(allianceColor == Colors::BLUE ? blue_alliance : red_alliance));
	}
	pathIter = 0;
}

void pathViewerTask() {
	while(true) {
		if(pathIter < pathDisplay.size() && pathDisplay.size() > 1) {
			lv_obj_clear_flag(autonRobot, LV_OBJ_FLAG_HIDDEN);
			lv_obj_set_pos(autonRobot, (2 * pathDisplay[pathIter].x) - 11, 130 - (2 * pathDisplay[pathIter].y));
			if(pathIter < pathDisplay.size() - 1) {
				lv_img_set_angle(autonRobot, 10 * (pathDisplay[pathIter].t));
				pros::delay(33 - 2.5 * sqrt(pathDisplay[pathIter].main));
			}
			if(pathIter == 1) pros::delay(500);
			pathIter++;
		} else if(pathIter >= pathDisplay.size()) {
			pros::delay(1000);
			resetViewer(false);
		}
		pros::delay(10);
	}
}

// // // // // // UI // // // // // //

//
// Main UI
//

void uiInit() {
	// Initialize style
	lv_style_init(&taki);
	lv_style_set_bg_color(&taki, theme_color);
	lv_style_set_outline_color(&taki, theme_accent);
	lv_style_set_text_color(&taki, theme_accent);
	lv_style_set_bg_opa(&taki, 255);
	lv_style_set_outline_width(&taki, 3);
	lv_style_set_border_width(&taki, 0);
	lv_style_set_text_font(&taki, &lv_font_montserrat_16);
	lv_style_set_radius(&taki, 0);

	// Set up flags
	lv_obj_clear_flag(main_tv, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_clear_flag(colorind, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_add_flag(pageUp[0], LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(pageDown[1], LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(pageUp[0], LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(pageDown[1], LV_OBJ_FLAG_CLICKABLE);

	// Add styles
	lv_obj_add_style(main_tv, &taki, LV_PART_MAIN);
	lv_obj_add_style(colorind, &taki, LV_PART_MAIN);
	lv_obj_set_style_bg_opa(main_tv, 255, LV_PART_MAIN);
	lv_obj_set_style_outline_opa(main_tv, 0, LV_PART_MAIN);

	// Load main screen
	lv_scr_load(main_tv);
	lv_obj_set_tile(main_tv, autoSelector, LV_ANIM_OFF);

	// Initialize screens
	autoSelectorInit();
	motorTempsInit();
	pathPlanningInit();

	// Set up ring indicator
	lv_obj_set_parent(colorind, autoSelector);
	lv_obj_set_size(colorind, 21, 21);
	lv_obj_set_style_radius(colorind, 15, LV_PART_MAIN);
	lv_obj_set_style_outline_width(colorind, 16, LV_PART_MAIN);
	lv_obj_set_style_bg_opa(colorind, 0, LV_PART_MAIN);
	lv_obj_set_pos(colorind, 436, 35);
	lv_obj_move_foreground(colorind);
}

//
// Auton Selector UI
//

static void selectAuton(lv_event_t* e) {
	AutonObj* getAuton = (AutonObj*)lv_event_get_user_data(e);
	lv_obj_t* target = lv_event_get_target(e);
	for(int i = 0; i < lv_obj_get_child_cnt(autonTable); i++) {
		lv_obj_t* auton = lv_obj_get_child(autonTable, i);
		lv_obj_clear_state(auton, LV_STATE_CHECKED);
	}
	lv_obj_add_state(target, LV_STATE_CHECKED);
	lv_label_set_text(autonDesc, ((*getAuton).desc).c_str());
	lv_obj_set_style_bg_color(autonDesc, lv_color_darken((*getAuton).color, 150), LV_PART_MAIN);
	lv_obj_set_style_img_recolor(autonOptions, lv_color_darken((*getAuton).color, 150), LV_PART_MAIN);
	auton_sel.selector_callback = (*getAuton).callback;
	resetViewer(true);
}

static void autonUpEvent(lv_event_t* e) { lv_obj_scroll_by_bounded(autonTable, 0, lv_obj_get_height(autonTable), LV_ANIM_ON); }

static void autonDownEvent(lv_event_t* e) { lv_obj_scroll_by_bounded(autonTable, 0, -lv_obj_get_height(autonTable), LV_ANIM_ON); }

static void delayEvent(lv_event_t* e) {
	delayBool = !delayBool;
	lv_obj_set_style_bg_opa(autonDelay, delayBool ? 255 : 0, LV_PART_MAIN);
}

static void ladderEvent(lv_event_t* e) {
	ladderBool = !ladderBool;
	lv_obj_set_style_bg_opa(autonLadder, ladderBool ? 255 : 0, LV_PART_MAIN);
	resetViewer(true);
}

static void angleCheckCloseEvent(lv_event_t *e) {
	aligning = false;
}

lv_event_cb_t AngleCheckCloseEvent = angleCheckCloseEvent;

static void angleCheckEvent(lv_event_t* e) {
	angleViewer = lv_msgbox_create(NULL, "check alignment", "0°", NULL, true);
	angleText = lv_msgbox_get_text(angleViewer);
	aligning = true;

	lv_obj_add_event_cb(lv_msgbox_get_close_btn(angleViewer), AngleCheckCloseEvent, LV_EVENT_PRESSED, NULL);
	lv_obj_add_style(lv_msgbox_get_close_btn(angleViewer), &taki, LV_PART_MAIN);
	lv_obj_add_style(angleViewer, &taki, LV_PART_MAIN);
	lv_obj_set_style_text_font(angleViewer, &lv_font_montserrat_48, LV_PART_MAIN);
	lv_obj_set_style_text_font(lv_msgbox_get_title(angleViewer), &lv_font_montserrat_14, LV_PART_MAIN);
	lv_obj_set_style_text_font(lv_msgbox_get_close_btn(angleViewer), &lv_font_montserrat_24, LV_PART_MAIN);
	lv_obj_set_width(angleViewer, 300);
	lv_obj_align(angleViewer, LV_ALIGN_CENTER, 0, 0);
}

lv_event_cb_t SelectAuton = selectAuton;
lv_event_cb_t AutonUpEvent = autonUpEvent;
lv_event_cb_t AutonDownEvent = autonDownEvent;
lv_event_cb_t DelayEvent = delayEvent;
lv_event_cb_t LadderEvent = ladderEvent;
lv_event_cb_t AngleCheckEvent = angleCheckEvent;

void autoSelectorInit() {
	// Add base styles
	lv_obj_add_style(autonTable, &taki, LV_PART_MAIN);
	lv_obj_add_style(autonTable, &taki, LV_PART_ITEMS);
	lv_obj_add_style(autonDesc, &taki, LV_PART_MAIN);
	lv_obj_add_style(autonField, &taki, LV_PART_MAIN);
	lv_obj_add_style(autonRobot, &taki, LV_PART_MAIN);
	lv_obj_add_style(autonUp, &taki, LV_PART_MAIN);
	lv_obj_add_style(autonDown, &taki, LV_PART_MAIN);
	lv_obj_add_style(autonDelay, &taki, LV_PART_MAIN);
	lv_obj_add_style(autonLadder, &taki, LV_PART_MAIN);

	// Set image sources and default text
	lv_img_set_src(autonField, &red_alliance);
	lv_img_set_src(autonOptions, &auton_options);
	lv_img_set_src(autonOptionsOutline, &auton_options_outline);
	lv_img_set_src(autonRobot, &robot);
	lv_label_set_text(autonDesc, "No auton selected");
	lv_label_set_text(autonUp, LV_SYMBOL_UP "\n" LV_SYMBOL_UP "\n" LV_SYMBOL_UP "\n" LV_SYMBOL_UP "\n" LV_SYMBOL_UP);
	lv_label_set_text(autonDown, LV_SYMBOL_DOWN "\n" LV_SYMBOL_DOWN "\n" LV_SYMBOL_DOWN "\n" LV_SYMBOL_DOWN "\n" LV_SYMBOL_DOWN);
	lv_obj_set_style_img_recolor_opa(autonOptions, 255, LV_PART_MAIN);

	// Set flags
	lv_obj_add_flag(autonUp, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(autonDown, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(autonDelay, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(autonLadder, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(autonField, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_flag(autonRobot, LV_OBJ_FLAG_HIDDEN);

	// Set sizes of objects
	lv_obj_set_size(autonTable, 160, 206);
	lv_obj_set_size(autonDesc, 288, 57);
	lv_obj_set_size(autonField, 288, 144);
	lv_obj_set_size(autonDelay, 22, 22);
	lv_obj_set_size(autonLadder, 22, 22);

	// Align and set positions of objects
	lv_obj_align(autonTable, LV_ALIGN_LEFT_MID, 5, 0);
	lv_obj_align(autonDesc, LV_ALIGN_TOP_RIGHT, -5, 17);
	lv_obj_align(autonField, LV_ALIGN_BOTTOM_RIGHT, -5, -17);
	lv_obj_align(autonOptions, LV_ALIGN_TOP_RIGHT, -64, 20);
	lv_obj_align(autonOptionsOutline, LV_ALIGN_TOP_RIGHT, -64, 20);
	lv_obj_align(autonDelay, LV_ALIGN_TOP_RIGHT, -65, 21);
	lv_obj_align(autonLadder, LV_ALIGN_TOP_RIGHT, -65, 48);
	lv_obj_align(autonUp, LV_ALIGN_CENTER, -64, -68);
	lv_obj_align(autonDown, LV_ALIGN_CENTER, -64, 68);

	// Set layers of objects
	lv_obj_move_foreground(autonOptions);
	lv_obj_move_foreground(autonOptionsOutline);

	// Modify styles
	lv_obj_set_style_bg_color(autonDelay, theme_accent, LV_PART_MAIN);
	lv_obj_set_style_bg_color(autonLadder, theme_accent, LV_PART_MAIN);

	lv_obj_set_style_text_opa(autonUp, 128, LV_STATE_PRESSED);
	lv_obj_set_style_text_opa(autonDown, 128, LV_STATE_PRESSED);
	lv_obj_set_style_bg_opa(autonRobot, 0, LV_PART_MAIN);
	lv_obj_set_style_bg_opa(autonDelay, 0, LV_PART_MAIN);
	lv_obj_set_style_bg_opa(autonLadder, 255, LV_PART_MAIN);

	lv_obj_set_style_outline_width(autonTable, 1, LV_PART_ITEMS);
	lv_obj_set_style_outline_width(autonField, 5, LV_STATE_PRESSED);
	lv_obj_set_style_outline_width(autonRobot, 0, LV_PART_MAIN);
	lv_obj_set_style_outline_width(autonDelay, 0, LV_PART_MAIN);
	lv_obj_set_style_outline_width(autonLadder, 0, LV_PART_MAIN);
	lv_obj_set_style_outline_width(autonUp, 0, LV_PART_MAIN);
	lv_obj_set_style_outline_width(autonDown, 0, LV_PART_MAIN);

	lv_obj_set_style_text_font(autonUp, &lv_font_montserrat_16, LV_PART_MAIN);
	lv_obj_set_style_text_font(autonDown, &lv_font_montserrat_16, LV_PART_MAIN);
	lv_obj_set_style_text_line_space(autonUp, -12, LV_PART_MAIN);
	lv_obj_set_style_text_line_space(autonDown, -12, LV_PART_MAIN);
	lv_obj_set_style_pad_all(autonDesc, 5, LV_PART_MAIN);
	lv_obj_set_style_pad_right(autonDesc, 85, LV_PART_MAIN);
	lv_obj_set_style_pad_hor(autonTable, 0, LV_PART_MAIN);

	lv_obj_set_scrollbar_mode(autonTable, LV_SCROLLBAR_MODE_OFF);

	// Add events
	lv_obj_add_event_cb(autonUp, AutonUpEvent, LV_EVENT_CLICKED, NULL);
	lv_obj_add_event_cb(autonDown, AutonDownEvent, LV_EVENT_CLICKED, NULL);
	lv_obj_add_event_cb(autonDelay, DelayEvent, LV_EVENT_CLICKED, NULL);
	lv_obj_add_event_cb(autonLadder, LadderEvent, LV_EVENT_CLICKED, NULL);
	lv_obj_add_event_cb(autonField, AngleCheckEvent, LV_EVENT_CLICKED, NULL);

	// Set up list
	for(int i = 0; i < auton_sel.autons.size(); i++) {
		lv_obj_t* new_auto = lv_list_add_btn(autonTable, NULL, (auton_sel.autons[i].name).c_str());
		lv_obj_add_style(new_auto, &taki, LV_PART_MAIN);
		lv_obj_set_style_text_font(new_auto, &pros_font_dejavu_mono_18, LV_PART_MAIN);
		lv_obj_set_style_bg_color(new_auto, lv_color_darken(auton_sel.autons[i].color, 150), LV_PART_MAIN);
		lv_obj_set_style_outline_width(new_auto, 1, LV_PART_MAIN);
		lv_obj_set_style_outline_width(new_auto, 4, LV_STATE_CHECKED);
		lv_obj_set_style_outline_width(new_auto, 6, LV_STATE_PRESSED);
		lv_obj_set_style_bg_opa(new_auto, 120, LV_STATE_CHECKED);
		lv_obj_set_style_bg_opa(new_auto, 60, LV_STATE_PRESSED);
		lv_obj_set_style_pad_hor(new_auto, 8, LV_PART_MAIN);
		lv_obj_add_event_cb(new_auto, SelectAuton, LV_EVENT_CLICKED, &auton_sel.autons[i]);
	}
}

//
// Motor Temperatures UI
//

void motorTempsInit() {}

//
// Path Planning UI
//

void pathPlanningInit() {}