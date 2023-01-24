/*
  rgb.c - RGB Status Light Plugin for CNC machines

  Copyright (c) 2021 JAC
  Version 1.0 - November 7, 2021

  For use with grblHAL: (Official GitHub) https://github.com/grblHAL
  Wiki (via deprecated GitHub location): https://github.com/terjeio/grblHAL/wiki/Compiling-GrblHAL

  Written by JAC for use with the Expatria grblHAL2000 PrintNC controller boards:
  https://github.com/Expatria-Technologies/grblhal_2000_PrintNC

  PrintNC - High Performance, Open Source, Steel Frame, CNC - https://wiki.printnc.info

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This RGB control plugin is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with GrblHAL.  If not, see <http://www.gnu.org/licenses/>.

  Copyright reserved by the author.

  *NOTE*: You must add this around line 217 in gcode.h before compiling for the first time:
  RGB_Inspection_Light = 356,         //!< 356 - M356 // ** Collides with Plasma ** On = 1, Off = 2, RGB white LED inspection light in RGB Plugin
*/

#include <string.h>
#include "driver.h"

#if STATUS_LIGHT_ENABLE // Declared in my_machine.h - you must add in the section with the included plugins

#include "grbl/protocol.h"
#include "grbl/hal.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"
#include "grbl/alarms.h"
#include "grbl/nuts_bolts.h"         // For delay_sec non-blocking timer function


// Declarations

// Available RGB colors possible with just relays
#define RGB_OFF     0 // All RGB Off
#define RGB_RED     1 // Red
#define RGB_GREEN   2 // Green
#define RGB_BLUE    3 // Blue
#define RGB_YELLOW  4 // Red + Green
#define RGB_MAGENTA 5 // Red + Bue
#define RGB_CYAN    6 // Green + Blue
#define RGB_WHITE   7 // Red + Green + Blue

// Blink times in ms
#define RGB_SLOW    1000
#define RGB_FAST    750
#define RGB_PULSE   500

// Override Lights - Inspection Light, Spindle On, Flood/Mist
// States for state machine - *DO NOT CHANGE ORDER HERE*
#define ST_INIT             0
#define ST_ILIGHT           1
#define ST_SPINDLE          2
#define ST_FLOOD            3
#define ST_MIST             4
#define ST_MCODEA           5
#define ST_MCODEB           6
#define ST_MCODEC           7
#define ST_PRECEDENCE_CHECK 8
#define ST_LONG_LOOP        9
#define ST_PRECEDENCE_SOLID 10
#define ST_SHORT_LOOP       11

// RGB Flash States
#define ST_NO_FLASH                     10
#define ST_ALARM_RAISED                 11
#define ST_ALARM_BLACK                  12
#define ST_ALARM_DETAIL                 13
#define ST_ALARM_INFORM                 14
#define ST_ALARM_HINT                   15
#define RGB_ILIGHT                      16
#define RGB_ILIGHT_SPINDLE              17
#define RGB_ILIGHT_SPINDLE_FL_MIST      18
#define RGB_SPINDLE                     19
#define RGB_SPINDLE_FL_MIST             20
#define RGB_FL_MIST                     21
#define RGB_CFLAG                       22
#define RGB_CYCLE_FLASH                 23
#define ST_ALARM_TRANSITION             24
#define RGB_ILIGHT_FL_MIST              25
#define ST_HOLD                         26
#define RGB_PRECEDENCE                  27
#define RGB_INIT_CONDITIONS             28
#define ST_HOLD_WITH_OVERRIDE           29
#define ST_HOLD_OVERRIDE_RGB_OFF        30
#define ST_HOLD_RGB_OFF                 31
#define ST_OVERRIDE_LOOP_EXIT           32

// Set preferred STATLE_IDLE light color, will be moving to a $ setting in future
static uint8_t RGB_IDLE = RGB_WHITE;         // Some people prefer RGB_WHITE for the idle color
static uint8_t inspection_light_on = 0;     // Indicates whether ILIGHT inspection light is on (1) or off (0)
static uint8_t base_port_out;               // Calculated starting point for assigning Aux Outputs to our plugin
static uint8_t base_port_in;                // Calculated starting point for assigning Aux Inputs to our plugin
static uint8_t red_port;                    // Aux out connected to a relay controlling the ground line for RED in an LED strip
static uint8_t green_port;                  // Aux out connected to a relay controlling the ground line for GREEN in an LED strip
static uint8_t blue_port;                   // Aux out connected to a relay controlling the ground line for BLUE in an LED strip
static uint8_t ilight_button_port = 0;      // Aux in connected to button to turn inspection light on/off, fix when refactoring input selection code
static uint8_t rgb_lstate = ST_NO_FLASH;    // Flag to track position in light flashing sequence
static uint8_t rgb_precedence = -1;         // For tracking ILIGHT or SPINDLE assert as light override
static uint8_t rgb_default_trigger = 0;     // Flag used for trigger default lights via OnStateChanged without STATE changing
static uint8_t cycle = 1;                   // Counter for use with ARCYCLE settings
static uint8_t hcycle = 1;                  // Hold Cycle Counter for use with STATE_HOLD light sequences
static sys_state_t last_state = -1;         // For tracking previous state.  Also used on first call after power-on to ensure trigger
static sys_state_t current_state;           // For storing the current state from sys.state via state_get()
static bool ovrr_init = 0;                  // Flag to indicate if ST_INIT was triggered in override lights function
unsigned long rgb_time = 0;
unsigned long current_timestamp;
//unsigned long atime, btime, ctime, nltime;

static on_state_change_ptr on_state_change;                 
static on_realtime_report_ptr on_realtime_report = NULL;   
static on_report_options_ptr on_report_options;            
static on_program_completed_ptr on_program_completed;
static driver_reset_ptr driver_reset;
static on_execute_realtime_ptr on_execute_realtime; // For real time loop insertion
static user_mcode_ptrs_t user_mcode;

typedef struct { // Structure to store the RGB bits
    uint8_t R;
    uint8_t G;
    uint8_t B;
} COLOR_LIST;

// Accessed as RGB_MASKS[RGB_YELLOW].R for the red value, RGB_MASKS[RGB_YELLOW].G for green etc.
static COLOR_LIST RGB_MASKS[8] = {
        { 0, 0, 0 },  // Off [0]
        { 1, 0, 0 },  // Red [1]
        { 0, 1, 0 },  // Green [2]
        { 0, 0, 1 },  // Blue [3]
        { 1, 1, 0 },  // Yellow [4]
        { 1, 0, 1 },  // Magenta [5]
        { 0, 1, 1 },  // Cyan [6]
        { 1, 1, 1 },  // White [7]
};

typedef struct { // Structure to store current and previous condition status for override lights
    uint16_t condition;
    uint8_t color;
    uint8_t curr;
    uint8_t prev;
} STATUS_LIST;

// Accessed as CONDITIONS[ST_ILIGHT].curr for the current value, .prev for the previous
static STATUS_LIST CONDITIONS[16] = {
        { ST_INIT,      RGB_OFF,        0, 0},     // ST INIT [0]
        { ST_ILIGHT,    RGB_WHITE,      0, 0},     // ST_ILGHT [1]
        { ST_SPINDLE,   RGB_RED,        0, 0},     // ST_SPINDLE [2]
        { ST_FLOOD,     RGB_MAGENTA,    0, 0},     // ST_FLOOD [3]
        { ST_MIST,      RGB_MAGENTA,    0, 0},     // ST_MIST [4]
        { ST_MCODEA,    RGB_OFF,        0, 0},     // ST_MCODEA [5] - For future use A-C
        { ST_MCODEB,    RGB_OFF,        0, 0},     // ST_MCODEB [6]
        { ST_MCODEC,    RGB_OFF,        0, 0},     // ST_MCODEC [7]
};

/* RGB Color mapping for STATEs and ALARMs - Should probably move to readme?
Red Solid               Caution: Spindle is on (overrides other states)
Red Flashing Slow       Alarm State
Red Flashing Fast       X axis fault hint

Green Solid             Jogging under user control ($J commands)
Green Flashing Slow     Motor Event
Green Flashing Fast     Y Axis fault hint

Blue Solid              Machine is energized and in an idle state
Blue Flashing Slow      Hard Limit Switch Event
Blue Flashing Fast      Z Axis fault hint

Yellow Solid            Homing Underway
Yelow Flashing Slow     Feed Hold, also Door Ajar
Yellow Flashing Fast    UNASSIGNED OR A Axis fault hint

Magenta Solid           GCode Being Executed, Spindle Off
Magenta Flashing Slow   Abort Cycle
Magenta Flashing Fast   UNASSIGNED OR B Axis fault hint

Cyan Solid              Probing
Cyan Flashing Slow      Homing approach event
Cyan Flashing Fast      UNASSIGNED OR C Axis fault hint

White Solid             Inspection Light
White Flashing Slow     Soft Limit Event
White Flashing Fast     Spindle Event
*/

typedef struct { // Structure to store the alarm code light indicator configuration
    uint8_t AINFORM;    // Color to inform operator of major alarm type, e.g. RGB_YELLOW for a homing related error
    uint8_t ADETAIL;    // (Optional) Color for alarm detail, e.g. RGB_GREEN for FailApproach, set to 0 to disable
    uint8_t AHINT;      // (Optional) Color for hint, e.g. Axis affected, set to 0 to disable
    uint8_t ARCYCLE;    // Number of RED cycles before each inform/detail/hint cycle
    int AD1;        // Alarm duration one
    int AD2;        // Alarm duration two
    int AD3;        // Alarm duration three 
} ALARM_CFG;


// Accessed as ALARM_LIGHTS[sys.alarm].value, ordered by alarm code from alarms.h
// All alarms start with solid RED flashing off based on the AD1 time interval
// Alarm lights must be in same order as alarm declarations in alarms.h, e.g. position 0 relates to Alarm_None
static ALARM_CFG ALARM_LIGHTS[] = {                                               // Did not delcare array size so new entries can be added and compiler will adjust       
        { RGB_RED, RGB_OFF, 0, 2, RGB_SLOW, RGB_SLOW, RGB_SLOW },         // Alarm_None = 0
        { RGB_BLUE, RGB_RED, RGB_GREEN, 3, RGB_SLOW, RGB_FAST, RGB_PULSE },        // Alarm_HardLimit = 1  Note: RGB_RED is a non-zero placeholder, actual Axis reported will be used
        { RGB_BLUE, RGB_WHITE, 0, 3, RGB_SLOW, RGB_FAST, RGB_PULSE },      // Alarm_SoftLimit = 2  
        { RGB_MAGENTA, RGB_RED, 0, 3, RGB_SLOW, RGB_SLOW, RGB_FAST },      // Alarm_AbortCycle = 3
        { RGB_CYAN, RGB_RED, 0, 3, RGB_SLOW, RGB_FAST, RGB_PULSE },        // Alarm_ProbeFailInitial = 4  Probe not in correct state to start probing
        { RGB_CYAN, RGB_BLUE, 0, 3, RGB_SLOW, RGB_FAST, RGB_PULSE },       // Alarm_ProbeFailContact = 5  Probe did not make contact in programmed time
        { RGB_YELLOW, RGB_RED, 0, 3, RGB_SLOW, RGB_FAST,RGB_PULSE },       // Alarm_HomingFailReset = 6
        { RGB_YELLOW, RGB_YELLOW, 0, 3, RGB_SLOW, RGB_FAST, RGB_PULSE },   // Alarm_HomingFailDoor = 7
        { RGB_YELLOW, RGB_GREEN, 0, 3, RGB_SLOW, RGB_FAST, RGB_PULSE },       // Alarm_FailPulloff = 8
        { RGB_YELLOW, RGB_CYAN, 0, 3, RGB_SLOW, RGB_FAST, RGB_FAST},       // Alarm_HomingFailApproach = 9
        { RGB_RED, RGB_BLUE, 0, 3, RGB_FAST, RGB_PULSE, RGB_PULSE },         // Alarm_EStop = 10
        { RGB_YELLOW, 0, 0, 3, RGB_FAST, RGB_FAST, RGB_PULSE},              // Alarm_HomingRequried = 11
        { RGB_BLUE, RGB_BLUE, 0, 3, RGB_SLOW, RGB_FAST, RGB_PULSE },       // Alarm_LimitsEngaged = 12
        { RGB_CYAN, RGB_WHITE, 0, 3, RGB_SLOW, RGB_FAST, RGB_PULSE },      // Alarm_ProbeProtect = 13  NOTE: ** Need more information on this event? ** Used White for now to differentiate
        { RGB_WHITE, RGB_MAGENTA, 0, 2, RGB_FAST, RGB_FAST, RGB_SLOW },    // Alarm_Spindle = 14
        { RGB_YELLOW, RGB_CYAN, 0, 3, RGB_SLOW, RGB_FAST, RGB_PULSE },     // Alarm_HomingFailAutoSquaringApproach = 15
        { RGB_RED, RGB_BLUE, 0, 2, RGB_FAST, RGB_FAST, RGB_PULSE },        // Alarm_SelftestFailed = 16
        { RGB_GREEN, RGB_RED, 0, 2, RGB_SLOW, RGB_FAST, RGB_PULSE },       // Alarm_MotorFault = 17
};

// For the flashing routine
static unsigned long state_start_timestamp = 0;     // In milliseconds

// Pin descriptions for setup as seen in $PINS command from grblHAL console
static const char *rgb_aux_out[] = {
    "LED Red",
    "LED Green",
    "LED Blue",
};

static const char *rgb_aux_in[] = {
    "BUTTON: Inspection Light",
};

// Decs for TEMP BUTTON testing
// Based on button debounce example found here: 
#define numOfInputs 1
//const int inputPins[numOfInputs] = {36u};

int ILIGHT_State = 0;
int inputState[numOfInputs];
int lastInputState[numOfInputs] = {false};
bool inputFlags[numOfInputs] = {false};
int inputCounters[numOfInputs];
long lastDebounceTime[numOfInputs] = {0};
long debounceDelay = 30;


// Functions

// Non-blocking ms delay function, &start_timestamp is a pointer so multiple functions can be using this simultaneously
// Return false if within request duration, true if duration ms has elapsed.
// boolean rgb_delay_ms(unsigned long start_timestamp, unsigned long duration) {
bool rgb_delay_ms(unsigned long duration) {
 
  unsigned long new_current_timestamp;

  new_current_timestamp = hal.get_elapsed_ticks();

    if (new_current_timestamp - state_start_timestamp >= duration) {
        state_start_timestamp = new_current_timestamp;
        return true;
    } 
    return false;
}

// Monitors for button press
void setInputFlags() {
    for(int i = 0; i < numOfInputs; i++) {
//    int reading = digitalRead(inputPins[i]);
    int reading = hal.port.wait_on_input(true, ilight_button_port, WaitMode_Immediate, 0.0f);
    if (reading != lastInputState[i]) {        
        if (rgb_delay_ms (debounceDelay)) {
            if (reading != inputState[i]) {
                inputState[i] = reading;
                if (inputState[i] == true) {
                    inputFlags[i] = true;
                }
            }
        }
        lastInputState[i] = reading;
        }
    }
}

void updateLEDState(int input) {
  // input 0 = State 0 and 1
  if(input == 0) {
    if(ILIGHT_State == 0) {
      ILIGHT_State = 1;
    }else{
      ILIGHT_State = 0;
    }
  // input 1 = State 2 to 6
  // This was from an example from the web and is to cycle through states in order rather than just on/off, not used yet
  }else if(input == 1) { // 2,3,4,5,6,2,3,4,5,6,2,
    if(ILIGHT_State == 0 || ILIGHT_State == 1 || ILIGHT_State > 5) {
      ILIGHT_State = 2;
    }else{
      ILIGHT_State++;
    }
  }
}

void resolveInputFlags() {
  for(int i = 0; i < numOfInputs; i++) {
    if(inputFlags[i] == true) {
      // Input Toggle Logic
      inputCounters[i]++;
      updateLEDState(i); 
      //printString(i);
      inputFlags[i] = false;
    }
  }
}

void resolveOutputs() {
  switch (ILIGHT_State) {
    case 0:
        if (inspection_light_on == 1) {
            inspection_light_on = 0;
        }
      break;
    case 1:
      if (inspection_light_on == 0) {
            inspection_light_on = 1;
        }
      break;
    default: 
        break;
  }
}

// Debug function to directly set a color outside of the standard function so we can use with delay() like a printf for debug
// Should add debug output to UART port for future debugging
static void rgb_debug (uint8_t rgb_debug_color) {
    hal.port.digital_out(red_port, RGB_MASKS[rgb_debug_color].R);
    hal.port.digital_out(green_port, RGB_MASKS[rgb_debug_color].G);
    hal.port.digital_out(blue_port, RGB_MASKS[rgb_debug_color].B);
}

// Physically sets the requested RGB light combination.
// Always sets all three LEDs to avoid unintended light combinations
static void rgb_set_led (uint8_t reqColor) { 
    static uint8_t currColor = 99;
    if ( currColor != reqColor) {
        currColor = reqColor;
        hal.port.digital_out(red_port, RGB_MASKS[reqColor].R);
        hal.port.digital_out(green_port, RGB_MASKS[reqColor].G);
        hal.port.digital_out(blue_port, RGB_MASKS[reqColor].B);
        state_start_timestamp = hal.get_elapsed_ticks();        // Update start time for state
    }
}

static void rgb_set_lstate (uint8_t newstate) {
    rgb_lstate = newstate;
    current_timestamp = hal.get_elapsed_ticks();
}

// M-Code handling code from Terje

// check - check if M-code is handled in this plugin.
// parameters: mcode - M-code to check for (some are predefined in user_mcode_t in grbl/gcode.h), use a cast if not.
// returns:    mcode if handled, UserMCode_Ignore otherwise (UserMCode_Ignore is defined in grbl/gcode.h).
static user_mcode_t check (user_mcode_t mcode)
{
    return mcode == RGB_Inspection_Light  // Must be added to gcode.h 
                     ? mcode                                                            // Handled by us.
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore); // If another handler present then call it or return ignore.
}

// validate - validate parameters
// parameters: gc_block - pointer to parser_block_t struct (defined in grbl/gcode.h).
//             gc_block->words - holds a bitfield of parameter words available.
//             If float values are NAN (Not A Number) this means they are not available.
//             If integer values has all bits set to 1 this means they are not available.
// returns:    status_code_t enum (defined in grbl/gcode.h): Status_OK if validated ok, appropriate status from enum if not.

static status_code_t validate (parser_block_t *gc_block, parameter_words_t *deprecated)
{
    status_code_t state = Status_GcodeValueWordMissing;

    switch(gc_block->user_mcode) {

        case RGB_Inspection_Light:
            if(gc_block->words.p && !isnan(gc_block->values.p))             // Check if P parameter value is supplied.
                state = Status_BadNumberFormat;                             // Return error if so.  Why?  P is valid as first variable?

            if(gc_block->words.q && isnan(gc_block->values.q))              // Check if Q parameter value is supplied.
                state = Status_BadNumberFormat;                             // Return error if not.

            if(state != Status_BadNumberFormat && gc_block->words.q) {      // Are required parameters provided?
                if(gc_block->values.q > 0.0f && gc_block->values.q <= 5.0f) // If Yes, is Q parameter value in range (1-5)?
                    state = Status_OK;                                      // If Yes - return ok status.
                else
                    state = Status_GcodeValueOutOfRange;                    // Else No - return error status.
                if(gc_block->words.q)                                       // If P parameter is present set
                    gc_block->values.p = 1.0f;                              // value to 1 for execution.
                gc_block->words.p = gc_block->words.q = Off;                // Claim parameters.
                gc_block->user_mcode_sync = true;                           // Optional: execute command synchronized
            }
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    // If not handled by us and another handler present then call it.
    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block, deprecated) : state;
}

// execute - execute M-code
// parameters: state - sys.state (bitmap, defined in system.h)
//             gc_block - pointer to parser_block_t struct (defined in grbl/gcode.h).
// returns:    -
static void execute (sys_state_t state, parser_block_t *gc_block) {

    bool handled = true;

    switch(gc_block->user_mcode) {

        case RGB_Inspection_Light: // To use, call mcode with M356 Q1 where 356 is your user MCode defined in gcode.h & Q is your variable (1-5)
            // do something: Q parameter value can be found in gc_block->values.q.
            //               P parameter has its value in gc_block->values.p set to 1 if present, NAN if not.
            if (gc_block->values.q == 1) {      
                //rgb_set_led(RGB_WHITE);           // M356 Q1 - Inspection light on
                ILIGHT_State = 1;                   // Set flag so IDLE, JOG, HOMING etc don't over ride light, ALARM does
            }
            else {
                if (gc_block->values.q == 2)        
                   // rgb_set_led(RGB_OFF);         // M356 Q2 - Inspection light off
                   ILIGHT_State = 0;                // Remove flag
            }
            break;

        default:
            handled = false;
            break;
    }


    if(!handled && user_mcode.execute)          // If not handled by us and another handler present
        user_mcode.execute(state, gc_block);    // then call it.
}


// END M-Code code from Terje

static void warning_msg (uint_fast16_t state)
{
    report_message("RGB plugin failed to initialize!", Message_Warning);
}

static void realtimeAlarmLightStates (void) {

    alarm_code_t alarm_code; // For Alarm_*  Only present it STATE_ALARM is asserted

    // ALARM STATE HANDLER
    //
    // This section handles providing light sequences for various STATE_ALARM and STATE_ESTOP conditions
    // These alarms have the highest precedence
    // Configuration is handled in ALARM_CONFIG & ALARM_LIGHTS structs at the top of the file
    //
    // Note: rgb_set_delay() is non-blocking

    alarm_code = (sys.alarm); // Get the alarm code

    // If Alarm is raised, turn on red light and initiate state machine loop
    if ( ( current_state == STATE_ALARM ) || ( current_state == STATE_ESTOP ) ) { 
            if ( (rgb_lstate == ST_NO_FLASH) && ( cycle <= ALARM_LIGHTS[alarm_code].ARCYCLE) ) {
                rgb_set_led(RGB_RED);
                if (rgb_delay_ms(ALARM_LIGHTS[alarm_code].AD2)) {
                    rgb_set_lstate(ST_ALARM_RAISED);
                }
            }           
            if  (rgb_lstate == ST_ALARM_RAISED) {         
                // Blink OFF, increment a counter so we get three RED/OFF sequences before signaling alarm type
                    if ( cycle <= ALARM_LIGHTS[alarm_code].ARCYCLE ) {
                        rgb_set_led(RGB_OFF);
                        if (rgb_delay_ms(ALARM_LIGHTS[alarm_code].AD2)) {
                            rgb_set_lstate(ST_ALARM_BLACK);
                            cycle++;
                        }   
                    }
            }

            // Transition to the regular information loop and reset the pre-amble cycle counter
            if ( cycle > ALARM_LIGHTS[alarm_code].ARCYCLE ) {
                rgb_set_led(RGB_OFF);
                    rgb_set_lstate(ST_ALARM_TRANSITION);
                    cycle = 1; // Don't move me
            }
    
            // Blink On - First step of alarm cycle 
            if ( (rgb_lstate == ST_ALARM_BLACK) && ( cycle <= ALARM_LIGHTS[alarm_code].ARCYCLE ) ) { // && ((long)(current_timestamp - state_start_timestamp) >= ALARM_LIGHTS[alarm_code].AD2))  {
                    rgb_set_led(RGB_RED);
                    if (rgb_delay_ms(ALARM_LIGHTS[alarm_code].AD2)) {            
                        rgb_set_lstate(ST_ALARM_RAISED);  // Loops back for the red pre-amble flashing
                    }
            }
            
            // Switch ON the Alarm INFORM light color for the major type of the alarm
            if  (rgb_lstate == ST_ALARM_TRANSITION) { // && ((long)(current_timestamp - state_start_timestamp) >= ALARM_LIGHTS[alarm_code].AD1)) {
                rgb_set_led(ALARM_LIGHTS[alarm_code].AINFORM);
                if (rgb_delay_ms(ALARM_LIGHTS[alarm_code].AD2)) {
                    rgb_set_lstate(ST_ALARM_INFORM);
                }
            }

            // If ADETAIL is not set for this alarm code, switch off and reset loop e.g. Alarm 11 Homing Req @ power-on is INFORM only
            if ( ( rgb_lstate == ST_ALARM_INFORM ) && (!(ALARM_LIGHTS[alarm_code].ADETAIL)) ) { //&& ((long)(current_timestamp - state_start_timestamp) >= ALARM_LIGHTS[alarm_code].AD2)) {
                rgb_set_led(RGB_OFF);
                if (rgb_delay_ms(ALARM_LIGHTS[alarm_code].AD2)) { 
                    rgb_set_lstate(ST_ALARM_BLACK);  // Reset Alarm loop
                }   
            }

            // Switch ON the Alarm Detail light color for the major type of alarm       
            if ((ALARM_LIGHTS[alarm_code].ADETAIL)) {  //&& ((long)(current_timestamp - state_start_timestamp) >= ALARM_LIGHTS[alarm_code].AD1)) {
                if (rgb_lstate == ST_ALARM_INFORM) {
                    rgb_set_led(ALARM_LIGHTS[alarm_code].ADETAIL);
                    if (rgb_delay_ms(ALARM_LIGHTS[alarm_code].AD3)) {
                        rgb_set_lstate(ST_ALARM_DETAIL);
                    }
                }
            }
                // Switch ON the Alarm Hint light color for the alarm detail
                if ( (ALARM_LIGHTS[alarm_code].ADETAIL) && (rgb_lstate == ST_ALARM_DETAIL) ) { //&& ((long)(current_timestamp - state_start_timestamp) >= ALARM_LIGHTS[alarm_code].AD2) ) {
                    rgb_set_led(ALARM_LIGHTS[alarm_code].AHINT);
                    if (rgb_delay_ms(ALARM_LIGHTS[alarm_code].AD3)) {
                        rgb_set_lstate(ST_ALARM_HINT);
                    }
                }
                
                // Blink OFF and reset the loop (note the amount of this OFF is actually the time in the first step of the loop)
                if ( (rgb_lstate == ST_ALARM_HINT) ) { // && ((long)(current_timestamp - state_start_timestamp) >= ALARM_LIGHTS[alarm_code].AD3) ) {
                    rgb_set_led(RGB_OFF);
                    if (rgb_delay_ms(ALARM_LIGHTS[alarm_code].AD2)) {
                        rgb_set_lstate(ST_ALARM_BLACK);  // Reset alarm loop
                    }
                }   
            
        }      
    }   

static void realtimeInputs() {  // Based on button input example given here: https://github.com/VRomanov89/EEEnthusiast/blob/master/03.%20Arduino%20Tutorials/01.%20Advanced%20Button%20Control/ButtonSketch/ButtonSketch.ino
    setInputFlags();
    resolveInputFlags();
    resolveOutputs();
}
  
static void realtimeConditionLights(void) {
    // Lights for conditions that are not contained within specific states e.g.
    // Inspection Light (WHITE solid)
    // Spindle indicator (RED solid)
    // Coolant & Mist (MAGENTA Solid)
    // Also warnings for these items within STATE_HOLD

    // If only used inside this function then do not use 'static', static reserves across entire lifetime
    uint8_t idx = 0;
    bool rgb_cond_changed = 0;       // Tracks if conditions changed since previous loop
    bool rgb_cond_asserted = 0;      // Tracks if any conditions are asserted this loop
    static sys_state_t prev_state = -1; // Track previous STATE_*

    current_timestamp = hal.get_elapsed_ticks(); // Get current millis ticks from from the timer
    current_state = state_get();    

    realtimeInputs();

    // Save previous status only if not in a blocking state
    for (idx = 0; idx <= 7; idx++ ) {
        CONDITIONS[idx].prev = CONDITIONS[idx].curr;
    }

    // Update current status
    CONDITIONS[ST_ILIGHT].curr = inspection_light_on;
    CONDITIONS[ST_SPINDLE].curr = hal.spindle.get_state().on;
    CONDITIONS[ST_FLOOD].curr = hal.coolant.get_state().flood;
    CONDITIONS[ST_MIST].curr = hal.coolant.get_state().mist;
    // Conditions for new MCodes, when added, go here

    for (idx = 0; idx <= 7; idx++) {
        if (CONDITIONS[idx].curr != CONDITIONS[idx].prev) {
            rgb_cond_changed = true;
        }
        if (CONDITIONS[idx].curr == On) {
            rgb_cond_asserted = true;
        }
    }

    // If we have new conditions, re-init the state machine
    if (rgb_cond_changed) {
        rgb_set_lstate(ST_INIT);
    }

    // If we have just entered STATE_HOLD, re-init
    if ( (current_state == STATE_HOLD) && (current_state != prev_state)) {
        rgb_set_lstate(ST_INIT);
    }

    prev_state = current_state;     // Reset state tracking

    // Override Light Conditions, State machine
    switch (rgb_lstate) {
        
        // Initialization
        case ST_INIT:

            // Re-init cycle counter
            hcycle = 1;
            rgb_precedence = 0;
            ovrr_init = true;

            // Is the ILight on?  If so, it will be the long light
            if (CONDITIONS[ST_ILIGHT].curr == On) {
                rgb_precedence = ST_ILIGHT;                
            }
            // If no ILIGHT, is the spindle on?  If so, it will be the long light
            else if (CONDITIONS[ST_SPINDLE].curr == On) {
                rgb_precedence = ST_SPINDLE;
            }

            // During STATE_HOLD
            // Use the main loop, or just flash yellow on and off if no conditions
            if (current_state == STATE_HOLD) {
                if (rgb_cond_asserted == true) {
                    rgb_set_lstate(ST_HOLD_WITH_OVERRIDE);
                }
                else{  // Otherwise if there are conditions, use the loop
                    rgb_set_lstate(ST_HOLD);
                }
            }
            // For all other conditions, enter the main loop
            else {
                if (rgb_cond_changed) {
                    rgb_set_lstate(ST_SHORT_LOOP);
                }
                else {
                rgb_set_lstate(ST_HOLD_WITH_OVERRIDE);  // Enter the override indicator light loop
                }
            }
            break;

        // NEW Top of loop
        // Special cases: HOLD, DOOR, TOOL_PROBE, CHECK_STATE?
        // If HOLD gets asserted, pre-amble a fast double pulse of yellow before the regular cycle
        case ST_HOLD_WITH_OVERRIDE:
            if ((current_state == STATE_HOLD) && (rgb_cond_asserted == true)) {
                if (hcycle <= 3) {
                    rgb_set_led(RGB_YELLOW);
                    if (rgb_delay_ms(RGB_PULSE)) {
                        rgb_set_lstate(ST_HOLD_OVERRIDE_RGB_OFF);
                    }
                }
                else {
                    rgb_set_lstate(ST_HOLD_OVERRIDE_RGB_OFF);
                }
            }
            else {
                rgb_set_lstate(ST_HOLD_OVERRIDE_RGB_OFF);
            }            
            break;

        // Flash off cycle for pre-amble state indicator
        case ST_HOLD_OVERRIDE_RGB_OFF:
            if ((current_state == STATE_HOLD) && (rgb_cond_asserted == true )) {
                if (hcycle <= 3) {
                    rgb_set_led(RGB_OFF);
                    if (rgb_delay_ms(RGB_PULSE)) {
                        hcycle++;
                        rgb_set_lstate(ST_HOLD_WITH_OVERRIDE); 
                    }
                }
                else { // If hcycle > 2, exit
                    hcycle = 1;  // Reset counter
                    rgb_set_lstate(ST_OVERRIDE_LOOP_EXIT);        
                }
            }
            else {
                rgb_set_lstate(ST_OVERRIDE_LOOP_EXIT);
            }
            break;

        case ST_OVERRIDE_LOOP_EXIT:
            // STATE HOLD TEST HERE
            // If this is the first time through after a re-init, jump to short flash cycle
            if (rgb_cond_asserted == true ) {
                if (current_state == STATE_HOLD) {  
                    if (ovrr_init) {
                        ovrr_init = false;
                        rgb_set_lstate(ST_SHORT_LOOP);     // Fast cycles through all active indicators for operator info
                        //hcycle = 0;  // Reset the cycle counter and proceed
                        break;
                    }
                    // Otherwise go through the regular loop which may include the long ILIGHT/SPINDLE then remaining fast indicators
                    else {                  
                        rgb_set_lstate(ST_PRECEDENCE_SOLID);
                        //hcycle = 0;  // Reset the cycle counter and proceed
                    }
                }
                else {
                    // If we are not in hold and a condition has changed, do the initial fast identification loop
                    if (rgb_cond_changed) {
                        rgb_set_lstate(ST_SHORT_LOOP);
                    }
                    // Otherwise go in to the long light loop
                    else {
                    rgb_set_lstate(ST_PRECEDENCE_SOLID);
                    }
                }
                // Another else here for when no condition asserted
            }
            break;

        // START of over-ride light state machine loop, each state continues thru to the next
        case ST_PRECEDENCE_SOLID: 
            // If we had a condition added or removed during last cycle, do fast flash
            if ((CONDITIONS[rgb_precedence].curr == On) && (rgb_cond_changed == false)) {
                rgb_set_led(CONDITIONS[rgb_precedence].color);
                if (rgb_delay_ms(RGB_SLOW * 4.5)) { // Length of time the precedence ILIGHT or Spindle light are on (todo make configurable)
                    rgb_set_lstate(ST_SHORT_LOOP);
                }
            }
            else {
                rgb_set_lstate(ST_SHORT_LOOP);
            }
            break;

        // Entry point for short flash condition indicators
        case ST_SHORT_LOOP:
            // If the ILIGHT is on (has precedence), start our short flash cycle with ST_SPINDLE
            // However, if we had a condition change state, do the fast indicate cycle once at the start of the change
            if ( (rgb_precedence > 0) && (rgb_cond_changed == false) ) {
                if  ( (rgb_precedence == ST_ILIGHT) && (current_state != STATE_HOLD) ) {
                    rgb_set_lstate(ST_SPINDLE);
                }
                // If the SPINDLE is on but ILIGHT is not (spindle has precedence), start our short flash with ST_FLOOD 
                else if ( (rgb_precedence == ST_SPINDLE) && (current_state != STATE_HOLD) ) {
                    rgb_set_lstate(ST_FLOOD);
                }
                // Otherwise, precedence set, go to the top of the loop because we pre-amble STATE_HOLD
                else{ 
                    rgb_set_lstate(ST_ILIGHT);
                }
            }
            else { // If neither are on, just proceed through all test cases
                rgb_set_lstate(ST_ILIGHT);
            }
            break;

        case ST_ILIGHT:
            if (CONDITIONS[ST_ILIGHT].curr == On ) {
                rgb_set_led(CONDITIONS[ST_ILIGHT].color);
                if (rgb_delay_ms(RGB_FAST)) {
                    rgb_set_lstate(ST_SPINDLE);
                }
            }
            else {
                rgb_set_lstate(ST_SPINDLE);
            }
            break;

        // Bug here where if SPINDLE is on (at idle state) and ILIGHT is turned on, does not do intro blink
        case ST_SPINDLE:
            if (CONDITIONS[ST_SPINDLE].curr == On ) {
                rgb_set_led(CONDITIONS[ST_SPINDLE].color);
                if (rgb_delay_ms(RGB_FAST)) {
                    rgb_set_lstate(ST_FLOOD);
                }
            }
            else {
                rgb_set_lstate(ST_FLOOD);
            }
            break;
        
        case ST_FLOOD:
            if (CONDITIONS[ST_FLOOD].curr == On ) {
                rgb_set_led(CONDITIONS[ST_FLOOD].color);
                if (rgb_delay_ms(RGB_FAST)) {
                    rgb_set_lstate(ST_MIST);
                }
            }
            else {
                rgb_set_lstate(ST_MIST);
            }
            break;
        
        case ST_MIST:
            if (CONDITIONS[ST_MIST].curr == On ) {                                
                rgb_set_led(CONDITIONS[ST_MIST].color);
                if (rgb_delay_ms(RGB_FAST)) {
                    rgb_set_lstate(ST_HOLD_WITH_OVERRIDE);
                }
            }
            else {
                rgb_set_lstate(ST_HOLD_WITH_OVERRIDE); // Return to top of loop
            }
            break;
        
        case ST_MCODEA:  // For potential future use with user MCodes
            //rgb_set_lstate(ST_MCODEB);
            break;
        
        case ST_MCODEB:
            //rgb_set_lstate(ST_MCODEC);
            break;
 
        case ST_MCODEC:
            // rgb_set_lstate() To top of loop
            break;

        // Special case not in main loop
        case ST_HOLD:  // STATE_HOLD with no override conditions, slow flash
            if ((current_state == STATE_HOLD) && (rgb_cond_asserted == false)) {
                rgb_set_led(RGB_YELLOW);
                    if (rgb_delay_ms(RGB_SLOW)) {
                        rgb_set_lstate(ST_HOLD_RGB_OFF);  // Loop to flash off
                    }
            }
            break; 

        case ST_HOLD_RGB_OFF:
            if ((current_state == STATE_HOLD) && (rgb_cond_asserted == false)) {
                rgb_set_led(RGB_OFF);
                    if (rgb_delay_ms(RGB_SLOW)) {
                        rgb_set_lstate(ST_HOLD); // Loop back to ST_HOLD               
                    }
            }                                             
            break;
        // End special cases

        default:
            //rgb_set_lstate(ST_NO_FLASH) ?;
            break;  // Since we fall thru, default case in the event we are called and don't have any conditions raised

    } // Closes Switch Statement
}

static void RGBUpdateState (sys_state_t state){
        current_state = state;
  
    // If our state has changed, or we want to force the lights to update, and no override light conditions exist
if ( ((current_state != last_state) || (rgb_default_trigger == 1)) && (!(inspection_light_on)) && (!(hal.spindle.get_state().on))  \
        && (!(hal.coolant.get_state().flood)) && (!hal.coolant.get_state().mist)  ) { 

        last_state = current_state;
    
        // No Over-rides or flash conditions
        switch (state) { // States with solid lights  *** These should use lookups
 
            // Chilling when idle, cool blue
            case STATE_IDLE:
                rgb_set_led(RGB_IDLE);
                rgb_set_lstate(ST_NO_FLASH);
                break; 

            // Running GCode (only seen if no M3 command for spindle on)
            case STATE_CYCLE:
                rgb_set_led(RGB_MAGENTA);
                rgb_set_lstate(ST_NO_FLASH);
                break; 
            
            // Investigate strange soft limits error in joggging
            case STATE_JOG:
                rgb_set_led(RGB_GREEN);
                rgb_set_lstate(ST_NO_FLASH);
                break; 

            // Would be nice to having homing be two colours as before, fast and seek - should be possible via real time thread
            case STATE_HOMING:
                rgb_set_led(RGB_YELLOW);
                rgb_set_lstate(ST_NO_FLASH);
                break;
            }
    }
}

static void RGBonStateChanged (sys_state_t state)
{
    RGBUpdateState(state);
    
    if (on_state_change)         // Call previous function in the chain.
        on_state_change(state);
}

static void realtimeIndicators (sys_state_t state) {
    // REALTIME STATUS INDICATOR LIGHTS
    //
    // This is the realtime function called from the main loop()
    
    current_state = state_get();          // Get the current machine STATE_*
    current_timestamp = hal.get_elapsed_ticks(); // Get current millis ticks from from the timer

    // LIGHTS THAT ASSERT ABOVE BASIC STATE STATUS
    if ((current_state == STATE_ALARM) || (current_state == STATE_ESTOP)) {
       realtimeAlarmLightStates();      // STATE_ALARM light sequences
    }

    // Conditional light sequences, SPINDLE, ILIGHT, FLOOD, MIST etc.
    // CHECK_MODE, SAFETY_DOOR, TOOL_CHANGE & SLEEP NOT ACCOUNTED FOR YET
    if ((current_state == STATE_IDLE) || (current_state == STATE_JOG) || (current_state == STATE_HOLD) || \
        (current_state == STATE_CYCLE) || (current_state == STATE_TOOL_CHANGE)) {
             realtimeConditionLights();    
     }
  
    // If no special lights are required, trigger the onStateChanged function to fall thru to default light status
    last_state = -1;
    rgb_default_trigger = 1;
    //onStateChanged(current_state); //This seems problematic because it breaks the call chain.
    RGBUpdateState(current_state); //replaced with this function that can be called in isolation.
 
    on_execute_realtime(state);         // Call previous function in the chain
}
// ON REALTIME REPORT - Data sent during ? query sent by sender to grblHAL
static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    // Do something when ? report is generated, coudld report which R/G/B is supposed to be on?

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}


static void onReportOptions (bool newopt) // Stock template
{
    on_report_options(newopt);  // Call previous function in the chain.

    if(!newopt)                 // Add info about us to the $I report.
        hal.stream.write("[PLUGIN:RGB Indicator Lights v1.0]" ASCII_EOL);
}

static void output_warning (uint_fast16_t state) // Sent if ports available < 3, see init function
{
    report_message("Three output ports are required for the RGB plugin!", Message_Warning);
}

// Tell the user about which ports are used for the output - not sent anywhere yet
// Is this needed since the info is in $PINS?  If so, add the new aux input details for ilight button & tool setter alarm
static void output_port (uint_fast16_t state)
{
    char msg[30];

    strcpy(msg, "R,G,B Ports: ");
    strcat(msg, uitoa(red_port));
    strcat(msg, uitoa(blue_port));
    strcat(msg, uitoa(green_port));

    report_message(msg, Message_Info);
}

// ON (Gcode) PROGRAM COMPLETION
static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    int cf_cycle = 0;

    // Job finished, wave the chequered flag.  Currently blocking, but as job is finished, is this an issue?
    while (cf_cycle <= 5) {
        rgb_set_led(RGB_WHITE);    
        rgb_set_lstate(RGB_CFLAG);
        hal.delay_ms(150, NULL);  // Changed from just delay() to make code more portable pre Terje IO
        rgb_set_led(RGB_OFF);    
        rgb_set_lstate(RGB_CFLAG);
        hal.delay_ms(150, NULL);
        cf_cycle++;
    }
    current_state = state_get();   

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
    
    cf_cycle = 0;
}

// DRIVER RESET - Release ports
static void driverReset (void)
{
    driver_reset();

    uint32_t idx_out = 2; // Ports count start at 0
    do {
        hal.port.digital_out(base_port_out + --idx_out, false);
    } while(idx_out);

    // Be aware that changing things here can lead to debugging challenges
    (inspection_light_on = 0);
}

// INIT FUNCTION - CALLED FROM plugins_init.h()
// void my_plugin_init() {
void status_light_init() {

    // CLAIM AUX OUTPUTS FOR RGB LIGHT RELAYS
    if(hal.port.num_digital_out >= 3) {

        hal.port.num_digital_out -= 3;  // Remove the our outputs from the list of available outputs
        base_port_out = hal.port.num_digital_out;

        if(hal.port.set_pin_description) {  // Viewable from $PINS command in MDI / Console
            uint32_t idx_out = 0;
            do {
                hal.port.set_pin_description(true, true, base_port_out + idx_out, rgb_aux_out[idx_out]);
                if      (idx_out == 0) { red_port = idx_out; }
                else if (idx_out == 1) { green_port = idx_out; } // NOTE - Fixed incorrect order (Blue was incorrectly here until Oct 21, 2021)
                else if (idx_out == 2) { blue_port = idx_out; }
                idx_out++;                
            } while(idx_out <= 2);
        //}

    // CLAIM AUX INPUT FOR INSPECTION LIGHT BUTTON
    /*if(hal.port.num_digital_in >= 2) {

        hal.port.num_digital_in -= 2;  // Remove the our inputs from the list of available inputs
        base_port_in = 0; //hal.port.num_digital_in; Check logic on assignment, seems wrong above too then?

        if(hal.port.set_pin_description) {  // Viewable from $PINS command in MDI / Console
            uint32_t idx_in = 0;
            do {
                hal.port.set_pin_description(true, false, base_port_in + idx_in, rgb_aux_in[idx_in]);
                if      (idx_in == 0) { ilight_button_port = idx_in; }
                else if (idx_in == 1) { toolsetter_alarm_port = idx_in; }
                idx_in++;                
            } while(idx_in <= 1);
        }*/

        last_state == STATE_CHECK_MODE;

        // Save away current HAL pointers so that we can use them to keep
        // any chain of M-code handlers intact.
        memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

        // Redirect HAL pointers to our code.
        hal.user_mcode.check = check;
        hal.user_mcode.validate = validate;
        hal.user_mcode.execute = execute;
      
        driver_reset = hal.driver_reset;                    // Subscribe to driver reset event
        hal.driver_reset = driverReset;

        on_report_options = grbl.on_report_options;         // Subscribe to report options event
        grbl.on_report_options = onReportOptions;           // Nothing here yet

        on_state_change = grbl.on_state_change;             // Subscribe to the state changed event by saving away the original
        grbl.on_state_change = RGBonStateChanged;              // function pointer and adding ours to the chain.

        on_realtime_report = grbl.on_realtime_report;       // Subscribe to realtime report events AKA ? reports
        grbl.on_realtime_report = onRealtimeReport;         // Nothing here yet

        on_program_completed = grbl.on_program_completed;   // Subscribe to on program completed events (lightshow on complete?)
        grbl.on_program_completed = onProgramCompleted;     // Checkered Flag for successful end of program lives here

        on_execute_realtime = grbl.on_execute_realtime;     // Subscribe to the realtime execution event
        grbl.on_execute_realtime = realtimeIndicators;      // Spindle monitoring, flashing LEDs etc live here

        state_start_timestamp = hal.get_elapsed_ticks();    // Initialize timer for flash loop, timestamp?
    }
} else
        protocol_enqueue_rt_command(warning_msg);
}
# endif