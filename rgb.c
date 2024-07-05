/*
  rgb.c - RGB Status Light Plugin for CNC machines

  Copyright (c) 2021 JAC
  Copyright (c) 2024 Terje Io - refactored to use the new RGB API when available, reduced MCU load significantly with new state handler for blinking etc...
                     NOTE: I do not know if this is 100% compatible with v1.

  Version 2.0 - February 4, 2024

  For use with grblHAL: (Official GitHub) https://github.com/grblHAL
  Wiki: https://github.com/grblHAL/core/wiki/Compiling-GrblHAL

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

  M356 -  On = 1, Off = 2, RGB white LED inspection light in RGB Plugin
*/

#include "driver.h"

#if STATUS_LIGHT_ENABLE == 1 // Declared in my_machine.h - you must add in the section with the included plugins

#include <string.h>
#include <math.h>

#include "grbl/protocol.h"
#include "grbl/hal.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"
#include "grbl/alarms.h"
#include "grbl/nuts_bolts.h"         // For delay_sec non-blocking timer function

// Declarations

// Available RGB colors possible with just relays
#define RGB_OFF     (rgb_color_t){ .R = 0, .G = 0, .B = 0 }
#define RGB_RED     (rgb_color_t){ .R = 50, .G = 0, .B = 0 } // Red
#define RGB_GREEN   (rgb_color_t){ .R = 0, .G = 50, .B = 0 } // Green
#define RGB_BLUE    (rgb_color_t){ .R = 0, .G = 0, .B = 50 } // Blue
#define RGB_YELLOW  (rgb_color_t){ .R = 50, .G = 50, .B = 0 } // Red + Green
#define RGB_MAGENTA (rgb_color_t){ .R = 50, .G = 0, .B = 50 } // Red + Bue
#define RGB_CYAN    (rgb_color_t){ .R = 0, .G = 50, .B = 50 } // Green + Blue
#define RGB_WHITE   (rgb_color_t){ .R = 50, .G = 50, .B = 50 } // Red + Green + Blue
#define RGB_INHERIT (rgb_color_t){ .W = 1 }

#define RGB_IDLE    RGB_WHITE
#define RGB_SPINDLE RGB_RED
#define RGB_COOLANT RGB_MAGENTA

// Blink times in ms
#define RGB_VERY_SLOW 4500
#define RGB_SLOW      1000
#define RGB_FAST      750
#define RGB_PULSE     500

typedef void (*blink_ptr)(uint32_t now);

typedef struct {
    sys_state_t state;
    rgb_color_t color;
    blink_ptr blinker;
    rgb_color_t dcolor;
} state_color_t;

typedef struct {
    rgb_color_t color;
    uint32_t delay;
} color_seq_t;

typedef struct { // Structure to store the alarm code light indicator configuration
    rgb_color_t color_major;    // Color to inform operator of major alarm type, e.g. RGB_YELLOW for a homing related error
    rgb_color_t color_detail;   // (Optional) Color for alarm detail, e.g. RGB_GREEN for FailApproach, set to RGB_OFF to disable
    rgb_color_t color_hint;     // (Optional) Color for hint, e.g. Axis affected, set to RGB_OFF to disable
    uint8_t major_cycles;       // Number of RED cycles before each inform/detail/hint cycle
    uint16_t delay[3];          // blink delays
} alarm_cfg_t;

static struct {
    uint32_t last_ms;
    uint_fast8_t seq;
    rgb_color_t base_color;
    blink_ptr call;
} blinker = {0};
static bool inspection_light = Off;         // Indicates whether ILIGHT inspection light is on or off
static uint8_t red_port;                    // Aux out connected to a relay controlling the ground line for RED in an LED strip
static uint8_t green_port;                  // Aux out connected to a relay controlling the ground line for GREEN in an LED strip
static uint8_t blue_port;                   // Aux out connected to a relay controlling the ground line for BLUE in an LED strip
static alarm_cfg_t *active_alarm;

static on_state_change_ptr on_state_change;
static on_report_options_ptr on_report_options;
static on_program_completed_ptr on_program_completed;
static on_execute_realtime_ptr on_execute_realtime;
static driver_reset_ptr driver_reset;
static user_mcode_ptrs_t user_mcode;
static rgb_set_color_ptr rgb_out;
static on_spindle_selected_ptr on_spindle_selected;
static spindle_set_state_ptr set_spindle_state;
#ifdef GRBL_ESP32
static esp32_spindle_off_ptr esp32_spindle_off;
#endif
static coolant_set_state_ptr set_coolant_state;
static coolant_state_t coolant_state = {0};
static spindle_state_t spindle_state = {0};

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

static void blink_alarm (uint32_t now);
static void blink_hold (uint32_t now);

static const rgb_color_t axis_colors[] = {
    RGB_RED,     // x
    RGB_GREEN,   // y
    RGB_BLUE,    // z
    RGB_YELLOW,  // a
    RGB_MAGENTA, // b
    RGB_CYAN,    // c
    RGB_WHITE,   // u
    RGB_OFF,     // v
};

static state_color_t state_color[] = {
    { STATE_IDLE, RGB_IDLE },
    { STATE_ALARM, RGB_RED, blink_alarm },
    { STATE_CHECK_MODE, RGB_WHITE },
    { STATE_HOMING, RGB_YELLOW,  },
    { STATE_CYCLE, RGB_GREEN },
    { STATE_HOLD, RGB_YELLOW, blink_hold },
    { STATE_JOG, RGB_GREEN },
    { STATE_SAFETY_DOOR, RGB_WHITE },
    { STATE_SLEEP, RGB_RED },
    { STATE_ESTOP, RGB_RED, blink_alarm },
    { STATE_TOOL_CHANGE, RGB_WHITE }
};

// Accessed as alarm_lights[sys.alarm].value, ordered by alarm code from alarms.h
// All alarms start with solid RED flashing off based on the AD1 time interval
// Alarm lights must be in same order as alarm declarations in alarms.h, e.g. position 0 relates to Alarm_None
static alarm_cfg_t alarm_lights[] = {
    { RGB_RED, RGB_OFF, RGB_OFF, 2, { RGB_SLOW, RGB_SLOW, RGB_SLOW } },          // Alarm_None = 0
    { RGB_BLUE, RGB_RED, RGB_GREEN, 3, { RGB_SLOW, RGB_FAST, RGB_PULSE } },      // Alarm_HardLimit = 1  Note: RGB_RED is a non-zero placeholder, actual Axis reported will be used
    { RGB_BLUE, RGB_WHITE, RGB_OFF, 3, { RGB_SLOW, RGB_FAST, RGB_PULSE } },      // Alarm_SoftLimit = 2
    { RGB_MAGENTA, RGB_RED, RGB_OFF, 3, { RGB_SLOW, RGB_SLOW, RGB_FAST } },      // Alarm_AbortCycle = 3
    { RGB_CYAN, RGB_RED, RGB_OFF, 3, { RGB_SLOW, RGB_FAST, RGB_PULSE } },        // Alarm_ProbeFailInitial = 4  Probe not in correct state to start probing
    { RGB_CYAN, RGB_BLUE, RGB_OFF, 3, { RGB_SLOW, RGB_FAST, RGB_PULSE } },       // Alarm_ProbeFailContact = 5  Probe did not make contact in programmed time
    { RGB_YELLOW, RGB_RED, RGB_OFF, 3, { RGB_SLOW, RGB_FAST,RGB_PULSE } },       // Alarm_HomingFailReset = 6
    { RGB_YELLOW, RGB_YELLOW, RGB_OFF, 3, { RGB_SLOW, RGB_FAST, RGB_PULSE } },   // Alarm_HomingFailDoor = 7
    { RGB_YELLOW, RGB_GREEN, RGB_OFF, 3, { RGB_SLOW, RGB_FAST, RGB_PULSE } },    // Alarm_FailPulloff = 8
    { RGB_YELLOW, RGB_CYAN, RGB_OFF, 3, { RGB_SLOW, RGB_FAST, RGB_FAST }},       // Alarm_HomingFailApproach = 9
    { RGB_RED, RGB_BLUE, RGB_OFF, 3, { RGB_FAST, RGB_PULSE, RGB_PULSE } },       // Alarm_EStop = 10
    { RGB_YELLOW, RGB_OFF, RGB_OFF, 3, { RGB_FAST, RGB_FAST, RGB_PULSE }},       // Alarm_HomingRequried = 11
    { RGB_BLUE, RGB_BLUE, RGB_OFF, 3, { RGB_SLOW, RGB_FAST, RGB_PULSE } },       // Alarm_LimitsEngaged = 12
    { RGB_CYAN, RGB_WHITE, RGB_OFF, 3, { RGB_SLOW, RGB_FAST, RGB_PULSE } },      // Alarm_ProbeProtect = 13  NOTE: ** Need more information on this event? ** Used White for now to differentiate
    { RGB_WHITE, RGB_MAGENTA, RGB_OFF, 2, { RGB_FAST, RGB_FAST, RGB_SLOW } },    // Alarm_Spindle = 14
    { RGB_YELLOW, RGB_CYAN, RGB_OFF, 3, { RGB_SLOW, RGB_FAST, RGB_PULSE } },     // Alarm_HomingFailAutoSquaringApproach = 15
    { RGB_RED, RGB_BLUE, RGB_OFF, 2, { RGB_FAST, RGB_FAST, RGB_PULSE } },        // Alarm_SelftestFailed = 16
    { RGB_GREEN, RGB_RED, RGB_OFF, 2, { RGB_SLOW, RGB_FAST, RGB_PULSE } }        // Alarm_MotorFault = 17
};

// GCC: __builtin_popcount(...)
// K&R version:
static uint_fast8_t bit_count (uint32_t n)
{
    uint_fast8_t count = 0;

    while(n) {
        n &= (n - 1);
        count++;
    }

    return count;
}

// Physically sets the requested RGB light combination.
// Always sets all three LEDs to avoid unintended light combinations

static void rgb_set_leds (uint16_t device, rgb_color_t color)
{
    hal.port.digital_out(red_port, color.R != 0);
    hal.port.digital_out(green_port, color.G != 0);
    hal.port.digital_out(blue_port, color.B != 0);
}

static void rgb_set_led (uint8_t device, rgb_color_t color)
{
    static rgb_color_t currColor = RGB_OFF;

    device = hal.rgb0.num_devices;

    if (currColor.value != color.value) {
        do {
            rgb_out(--device, (currColor = color));
        } while(device);
        if(rgb_out != rgb_set_leds && hal.rgb0.write)
            hal.rgb0.write();
    }
}

static void blink_hold_spindle (uint32_t now);

// Starts blink if blinker != NULL
static void start_blink (rgb_color_t color, blink_ptr blinker_fn)
{
    blinker.seq = 0;
    blinker.last_ms = hal.get_elapsed_ticks();
    rgb_set_led(0, color);
    blinker.call = blinker_fn;
}

static void blink_hint (uint32_t now)
{
    if(now - blinker.last_ms > active_alarm->delay[2])
        start_blink(blinker.base_color, blink_alarm);
}

static void blink_detail (uint32_t now)
{
    if(now - blinker.last_ms > active_alarm->delay[1]) {
        if(active_alarm->color_hint.value != (RGB_OFF).value)
            start_blink(active_alarm->color_hint, blink_hint);
        else
            start_blink(blinker.base_color, blink_alarm);
    }
}

static void blink_inform (uint32_t now)
{
    if(now - blinker.last_ms > active_alarm->delay[1]) {
        if(active_alarm->color_detail.value != (RGB_OFF).value)
            start_blink(active_alarm->color_detail, blink_detail);
        else
            start_blink(blinker.base_color, blink_alarm);
    }
}

static void blink_alarm (uint32_t now)
{
    if(now - blinker.last_ms > active_alarm->delay[1]) {
        blinker.last_ms = now;
        if(blinker.seq++ == active_alarm->major_cycles * 2) {
            blinker.seq = 0;
            if(active_alarm->color_major.value != (RGB_OFF).value)
                start_blink(active_alarm->color_major, blink_inform);
            else
                rgb_set_led(0, blinker.base_color);
        } else
            rgb_set_led(0, blinker.seq & 1 ? RGB_OFF : blinker.base_color);
    }
}

static void blink_coolant_spindle (uint32_t now)
{
    static const color_seq_t seq[] = {
        { RGB_INHERIT, RGB_PULSE }
    };

    if(now - blinker.last_ms > seq[blinker.seq].delay)
        start_blink(blinker.base_color, blink_hold_spindle);
}

static void blink_spindle (uint32_t now)
{
    static const color_seq_t seq[] = {
        { RGB_INHERIT, RGB_VERY_SLOW }
    };

    if(now - blinker.last_ms > seq[blinker.seq].delay) {
        if(coolant_state.value)
            start_blink(RGB_COOLANT, blink_coolant_spindle);
        else
            start_blink(blinker.base_color, blink_hold);
    }
}

static void blink_hold (uint32_t now)
{
    static const color_seq_t seq[] = {
        { RGB_INHERIT, RGB_SLOW },
        { RGB_OFF, RGB_SLOW }
    };

    if(now - blinker.last_ms > seq[blinker.seq].delay) {
        blinker.last_ms = now;
        if(++blinker.seq == sizeof(seq) / sizeof(color_seq_t))
            blinker.seq = 0;

        rgb_set_led(0, seq[blinker.seq].color.W ? blinker.base_color : seq[blinker.seq].color);
    }
}

static void blink_coolant_hold (uint32_t now)
{
    static const color_seq_t seq[] = {
        { RGB_INHERIT, RGB_FAST }
    };

    if(now - blinker.last_ms > seq[blinker.seq].delay)
        start_blink(blinker.base_color, blink_hold_spindle);
}

static void blink_hold_spindle (uint32_t now)
{
    static const color_seq_t seq[] = {
        { RGB_INHERIT, RGB_PULSE },
        { RGB_OFF, RGB_PULSE },
        { RGB_INHERIT, RGB_PULSE },
        { RGB_OFF, RGB_PULSE },
        { RGB_INHERIT, RGB_PULSE },
        { RGB_OFF, RGB_PULSE },
        { RGB_SPINDLE, RGB_VERY_SLOW }
    };

    if(now - blinker.last_ms > seq[blinker.seq].delay) {
        blinker.last_ms = now;
        if(++blinker.seq == sizeof(seq) / sizeof(color_seq_t)) {
            blinker.seq = 0;
            if(coolant_state.value) {
                start_blink(RGB_COOLANT, blink_coolant_hold);
                return;
            }
        }
        rgb_set_led(0, seq[blinker.seq].color.W ? blinker.base_color : seq[blinker.seq].color);
    }
}

static void blink_hold_coolant (uint32_t now)
{
    static const color_seq_t seq[] = {
        { RGB_INHERIT, RGB_PULSE },
        { RGB_OFF, RGB_PULSE },
        { RGB_INHERIT, RGB_PULSE },
        { RGB_OFF, RGB_PULSE },
        { RGB_INHERIT, RGB_PULSE },
        { RGB_OFF, RGB_PULSE },
        { RGB_COOLANT, RGB_FAST }
    };

    if(now - blinker.last_ms > seq[blinker.seq].delay) {
        blinker.last_ms = now;
        if(++blinker.seq == sizeof(seq) / sizeof(color_seq_t))
            blinker.seq = 0;
        rgb_set_led(0, seq[blinker.seq].color.W ? blinker.base_color : seq[blinker.seq].color);
    }
}

static void rgb_state_changed (sys_state_t state)
{
    if(!inspection_light || (state & (STATE_ALARM|STATE_ESTOP))) {

        uint_fast8_t i = sizeof(state_color) / sizeof(state_color_t);

        do {
            if(state_color[--i].state == state) {

                switch(state) {

                    case STATE_IDLE:
                    case STATE_CYCLE:
                    case STATE_JOG:
                        if(spindle_state.on) {
                            state_color[i].color = RGB_SPINDLE;
                            state_color[i].blinker = coolant_state.value ? blink_spindle : NULL;
                        } else {
                            state_color[i].color = coolant_state.value ? RGB_COOLANT : state_color[i].dcolor;
                            state_color[i].blinker = NULL;
                        }
                        break;

                    case STATE_HOLD:
                        if(spindle_state.on)
                            state_color[i].blinker = blink_hold_spindle;
                        else
                            state_color[i].blinker = coolant_state.value ? blink_hold_coolant : blink_hold;
                        break;

                    case STATE_ALARM:
                        inspection_light = Off;
                        active_alarm = &alarm_lights[sys.alarm >= sizeof(alarm_lights) / sizeof(alarm_cfg_t) ? 0 : sys.alarm];
                        if(sys.alarm == Alarm_HardLimit) {
                            int8_t axis;
                            axes_signals_t hard_limits = limit_signals_merge(sys.last_event.limits);
                            if(bit_count(hard_limits.mask) == 1 && (axis = ffsl(hard_limits.mask)) != -1)
                                active_alarm->color_detail = axis_colors[axis];
                            else
                                active_alarm->color_detail = RGB_WHITE; // ??
                        }
                        break;

                    case STATE_ESTOP:
                        inspection_light = Off;
                        active_alarm = &alarm_lights[10];
                        break;
                }

                start_blink((blinker.base_color = state_color[i].color), state_color[i].blinker);
                break;
            }

        } while(i);

    } else { // Inspection light is on
        blinker.call = NULL;
        rgb_set_led(0, RGB_WHITE);
    }
}

static void blink_completed (uint32_t now)
{
    static const color_seq_t seq[] = {
        { RGB_INHERIT, 150 },
        { RGB_OFF, 150 },
        { RGB_INHERIT, 150 },
        { RGB_OFF, 150 },
        { RGB_INHERIT, 150 },
        { RGB_OFF, 150 },
        { RGB_INHERIT, 150 },
        { RGB_OFF, 150 },
        { RGB_INHERIT, 150 },
        { RGB_OFF, 150 }
    };

    if(now - blinker.last_ms > seq[blinker.seq].delay) {
        blinker.last_ms = now;
        if(++blinker.seq == sizeof(seq) / sizeof(color_seq_t))
            rgb_state_changed(state_get());
        else
            rgb_set_led(0, seq[blinker.seq].color.W ? blinker.base_color : seq[blinker.seq].color);
    }
}

static void onStateChanged (sys_state_t state)
{
    static sys_state_t state_prev = STATE_CHECK_MODE;

    if(state != state_prev) {
        rgb_state_changed(state);
        state_prev = state;
    }

    if (on_state_change)
        on_state_change(state);
}

static void onExecuteRealtime (sys_state_t state)
{
    static uint32_t last_ms = 0;

    if(blinker.call) {

        uint32_t ms = hal.get_elapsed_ticks();

        if(ms != last_ms)
            blinker.call((last_ms = ms));
    }

    on_execute_realtime(state);
}

// ON (Gcode) PROGRAM COMPLETION
static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    start_blink((blinker.base_color = RGB_WHITE), blink_completed);

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

static void coolantSetState (coolant_state_t state)
{
    coolant_state = state;

    rgb_state_changed(state_get());

    set_coolant_state(state);
}

static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    spindle_state = state;

    rgb_state_changed(state_get());

    set_spindle_state(spindle, state, rpm);
}

#ifdef GRBL_ESP32

static void spindleOff (spindle_ptrs_t *spindle)
{
    spindle_state.value = 0;

    rgb_state_changed(state_get());

    esp32_spindle_off(spindle);
}

#endif

static void onSpindleSelected (spindle_ptrs_t *spindle)
{
    set_spindle_state = spindle->set_state;
    spindle->set_state = spindleSetState;

#ifdef GRBL_ESP32
    if(spindle->esp32_off) {
        esp32_spindle_off = spindle->esp32_off;
        spindle->esp32_off = spindleOff;
    }
#endif

    if(on_spindle_selected)
        on_spindle_selected(spindle);
}

static void driverReset (void)
{
    driver_reset();

    inspection_light = Off;

    rgb_set_led(0, RGB_OFF);
}

// M356 - inspection light: Q1 on, Q2 off

static user_mcode_t check (user_mcode_t mcode)
{
    return mcode == RGB_Inspection_Light
                     ? mcode                                                            // Handled by us.
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore); // If another handler present then call it or return ignore.
}

static status_code_t validate (parser_block_t *gc_block, parameter_words_t *deprecated)
{
    status_code_t state = Status_GcodeValueWordMissing;

    switch(gc_block->user_mcode) {

        case RGB_Inspection_Light:

            if(gc_block->words.q && isnan(gc_block->values.q))              // Check if Q parameter value is supplied.
                state = Status_BadNumberFormat;                             // Return error if not.

            if(state != Status_BadNumberFormat && gc_block->words.q) {      // Are required parameters provided?
                if(gc_block->values.q > 0.0f && gc_block->values.q <= 2.0f) // If Yes, is Q parameter value in range (1-2)?
                    state = Status_OK;                                      // If Yes - return ok status.
                else
                    state = Status_GcodeValueOutOfRange;                    // Else No - return error status.
                gc_block->words.q = Off;                                    // Claim parameter.
                gc_block->user_mcode_sync = true;                           // Optional: execute command synchronized
            }
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block, deprecated) : state;
}

static void execute (sys_state_t state, parser_block_t *gc_block)
{
    bool handled = true;

    switch(gc_block->user_mcode) {

        case RGB_Inspection_Light:
            inspection_light = gc_block->values.q == 1.0f;
            rgb_state_changed(state_get());
            break;

        default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)          // If not handled by us and another handler present
        user_mcode.execute(state, gc_block);    // then call it.
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);  // Call previous function in the chain.

    if(!newopt)                 // Add info about us to the $I report.
        hal.stream.write("[PLUGIN:RGB Indicator Lights v2.0]" ASCII_EOL);
}

void status_light_init (void)
{
    bool ok;

    if((ok = hal.rgb0.out != NULL && hal.rgb0.cap.R > 0 && hal.rgb0.cap.G > 0 && hal.rgb0.cap.B > 0)) {
        rgb_out = hal.rgb0.out;
        hal.rgb0.cap.value = 0; // Claim RGB output...
    } else if((ok = hal.port.num_digital_out >= 3)) {

        // CLAIM AUX OUTPUTS FOR RGB LIGHT RELAYS
        uint8_t base_port = hal.port.num_digital_out - 3;

        red_port = base_port;
        green_port = base_port + 1;
        blue_port = base_port + 2;

        ioport_claim(Port_Digital, Port_Output, &red_port, "LED Red");
        ioport_claim(Port_Digital, Port_Output, &green_port, "LED Green");
        ioport_claim(Port_Digital, Port_Output, &blue_port, "LED Blue");

        rgb_out = rgb_set_leds;
        hal.rgb0.num_devices = 1;
    }

    if(ok) {

        uint_fast8_t i = sizeof(state_color) / sizeof(state_color_t);

        do {
            i--;
            state_color[i].dcolor = state_color[i].color;
        } while(i);

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
        grbl.on_state_change = onStateChanged;              // function pointer and adding ours to the chain.

        on_program_completed = grbl.on_program_completed;   // Subscribe to on program completed events (lightshow on complete?)
        grbl.on_program_completed = onProgramCompleted;     // Checkered Flag for successful end of program lives here

        on_spindle_selected = grbl.on_spindle_selected;
        grbl.on_spindle_selected = onSpindleSelected;

        set_coolant_state = hal.coolant.set_state;
        hal.coolant.set_state = coolantSetState;

        on_execute_realtime = grbl.on_execute_realtime;     // Subscribe to the realtime execution event
        grbl.on_execute_realtime = onExecuteRealtime;       // Flashing LEDs live here

        if(hal.rgb0.set_intensity)
            hal.rgb0.set_intensity(63);
    } else
        protocol_enqueue_foreground_task(report_warning, "RGB plugin failed to initialize!");
}

#endif
