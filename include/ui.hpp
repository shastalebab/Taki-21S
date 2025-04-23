#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "autons.hpp"

const lv_color32_t theme_color = lv_color_hex(0x00060a);
const lv_color32_t theme_accent = lv_color_hex(0xe3f4ff);
const lv_color32_t red = lv_color_hex(0xaa2f17);
const lv_color32_t blue = lv_color_hex(0x1744aa);
const lv_color32_t gray = lv_color_hex(0x575757);

// Auton selector
class AutonObj {
    public:
        function<void()> callback = testauto;
        string name = "no name";
        string desc = "no description";
        lv_color32_t color = theme_color;
};

class AutonSel {
    public:
        vector<AutonObj> autons = {};
        function<void()> selector_callback = testauto;
        void selector_populate(vector<AutonObj> auton_list);
};

extern AutonSel auton_sel;
extern bool delayBool;
extern bool ladderBool;

// Motor Temperatures
void motorTempsTask();

// Path Planning
void pathViewerTask();

// Main UI
extern lv_obj_t* main_tv;
extern lv_obj_t* colorind;
extern lv_obj_t* autoSelector;
extern lv_obj_t* motorTemps;
extern lv_obj_t* pathPlanning;
void uiInit();

// Auton Selector UI
void autoSelectorInit();

// Motor Temperatures UI
void motorTempsInit();


// Path Planning UI
void pathPlanningInit();