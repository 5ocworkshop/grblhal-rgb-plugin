'DATRON LIKE' RGB INDICATOR LIGHTS
A Plugin for grblHAL
November 7, 2021
Version 1.0 

This plugin displays visual status by using the 7 different colors that can easily be
careted with a strip of RGB LED lights mounted under/behind the X gantry of a CNC machine.

I have mine mounted on a strip of wood that is angled at 45 degrees and shines the center
of the light the center of the gantry. [video link forthcoming]

This plugin written for the Teensy 4.1 based open source grblHAL2000 board by Expatria,
designed for the PrintNC DIY CNC community:
https://github.com/Expatria-Technologies/grblhal_2000_PrintNC

The code should be easily portable to other grblHAL platforms with Aux Out ports.

HOW TO:
-------

1) Make a new src directory called "rgb" in the same location as your other grblHAL plugin directories and place rgb.c in it
2) Edit gcode.h to add the custom MCode 356.  I added this line around line 217 in gcode.h:

    RGB_Inspection_Light = 356,         //!< 356 - M356 // ** Collides with Plasma ** On = 1, Off = 2, RGB white LED inspection light in RGB plugin

3) To enable the plugin, edit my_machine.h and add (under //#define FANS_ENABLE):
    #define RGB_ENABLE          1 // RGB Status Light Plugin, uses three aux outs to provide visual status via RGB LED light strip

4) To activate the plugin so it initializes when grblHAL loads, edit grbl/plugins_init.h and add the following
   at the bottom but ABOVE the odometer statement and above the "my_plugin_init();" line:

    #if RGB_ENABLE
        extern void rgb_init (void);
        rgb_init();
    #endif

5) Compile and flash your machine and enjoy.

CURRENTLY WORKING:
------------------

Basic State Status Lights:

MACHINE STATE       COLOR
-------------------------------------
Idle                Cool Blue (Solid)   -> Machine is ready  (Can be set to White if you prefer, see declarations in rgb.c)
Homing              Yellow (Solid)      -> Machine is inside HOMING state (other ops blocked)
Jogging             Green (Solid)       -> Machine is in motion, no spindle on
Cycle Running       Magenta (Solid)     -> GCode is being executed
Hold                Yellow (Flashing)   -> Machine is in hold state (no conditions present)
Alarm               3 Red Flashes followed by various indicators for alarm detail
E-Stop              2 Red flashes then red/blue flashes

CONDITIONS          COLOR
-------------------------------------
Spindle On          Red (Solid) [overrides idle/homing/jogging/cycle state lights]
Flood & Mist        Magenta (may switch to Cyan to avoid conflict with cycle, or may flash)
Inspection Lights   White (accesed via MCode 356), intend to add physical momentary button too
    M356 Q1 (on)    Can be controlled by a physical button pulled to ground on auxin0
    M356 Q2 (off)
                    
** You can use the $PINS command in the MDI to confirm the AUX Out assignments worked **

MCODE:
------
You need to add the mcode to your gcode.h file around line 217
N.B. The specific number chosen may change in future versions.

Around line 217, add this:
RGB_Inspection_Light = 356,         //!< 356 - M356 // ** Collides with Plasma ** On = 1, Off = 2, RGB white LED inspection light in RGB Plugin


ALARMS:
-------
Alarms have multiple levels.  INFORM, INFO and DETAIL.

For example, when the machine first powers on, if you are configured to require homing you will see

3 x Slow Red Flashes followed by 1 Yellow -> Indicates Alarm:Homing Required

In the above example, Yellow is the INFORM level.  Other alarms have more levels and detail.
There is a detailed table for alarm configuration in the structure at the top of the plugin.  WIP.


SPINDLE/INSPECTION LIGHTS:
--------------------------

The Inspection Light (White light) takes precedence over all other non-Alarm conditions.
However, in order to support safe operation, every 5s it will quick flash RED to indicate if
the SPINDLE is also on, and MAGENTA if the FLOOD/MIST is on.

You do not want to be drawn to the machine to look at something and forget the spindle is on.

If the Inspection Light is not on, and the Spindle is on in any state except alarm, the light
will be solid RED.  Every 5s it will flash to MAGENTA if FLOOD/MIST is on.


ON PROGRAM COMPLETION:
----------------------
When a gcode file completes and sends an M30 command, a chequered flag light pattern rapid flashes
for ~1.5s to provide a visual indication a program has completed.  You can try it by just sending M30
in the MDI.

Be aware, at the moment (and only this pattern) is a blocking operation.  It will be made non-blocking
soon but as it is at program end it should be ok for testing.
PHYSICAL WIRING (Amazon Links to parts below):
---------------
The plugin uses three of the four Aux Outputs on the grblHAL2000 board

I have mine wired up us:

Aux0 -> Control signal to relay controlling RED ground line on LED light strip
Aux1 -> Control signal to relay controlling GREEN ground line on LED light strip
Aux2 -> Control signal to relay controlling BLUE ground line on LED light strip

I have a PC 12V power supply currently supplying the power but will be moving to the 12V line in the Cisco PSU.


AMAZON LINKS:
-------------

I was originally using this relay (ground trigger):
https://www.amazon.ca/SunFounder-Channel-Arduino-Raspberry-Pi520141421293SunFounder/dp/B00E0NSORY/

But I switched to this 12V relay so all my accessories are 12V in my electronics enclosure:
https://www.amazon.ca/gp/product/B07WWD2WTK/

I am currently using lights similar to these (make sure they are 12V when ordering):
https://www.amazon.ca/gp/product/B07TD8SJQT/ Also I recommend getting the "weatherpoof" ones
so the circuit isn't exposed to chips from your machine.

TO DO:
------
Please see todo.txt