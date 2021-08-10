'DATRON LIKE' RGB INDICATOR LIGHTS
A Plugin for grblHAL
August 10, 2021
Version .7 

** Work in progress, use at your own risk **

A plugin to control a strip of RGB LED lights mounted under/behind the X gantry.
I have mine mounted on a strip of wood that is angled at 45 degrees and shines the center
of the light essentially under the center of the gantry.

This was written for the Teensy 4.1 based grblHAL2000 board by Expatrica,
designed for the PrintNC community. The code should be portable to other grblHAL platforms.

CURRENTLY WORKING:
------------------

Basic State Status Lights:

MACHINE STATE       COLOR
-------------------------------------
Idle                Cool Blue (Solid)   -> Machine is ready
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
    M356 Q1 (for on)
    M356 Q2 (for off)

** You can use the $PINS command in the MDI to confirm the AUX Out assignments worked **

MCODE:
------
You need to add the mcode to you gcode.h file around line 217
N.B. The specific number chosen may change in future versions.

Around line 217, add this:
RGB_Inspection_Light = 356,         //!< 356 - M356 // ** Collides with Plasma ** On = 1, Off = 2, RGB white LED inspection light in RGB Plugin


ALARMS:
-------
Alarms have multiple levels.  INFORM, INFO and DETAIL.

For example, when the machine first powers on, if it requires homing you will see

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

TO DO:
------
Detailed to do list in the source code.
The plugin does not current report any status back through the UI but that is planned.


HOW TO:
-------

1) Place the my_plugin.c file in your src/grbl directory for your grblhal build
2) Edit gcode.h to add the custom MCode 356.  Example in my repo, added this line around line 217

RGB_Inspection_Light = 356,         //!< 356 - M356 // ** Collides with Plasma ** On = 1, Off = 2, RGB white LED inspection light in RGB plugin

It should just compile.

** NOTE: I haven't found where i did it, but I did need to enable the Aux ports in one of the my_machine or similar files. **


PHYSICAL WIRING (Amazon Links to parts below):
---------------
The plugin uses three of the four Aux Outputs on the grblHAL2000 board

I have mine wired up us:

Aux0 -> Control signal to relay controlling RED ground on LED light strip
Aux1 -> Control signal to relay controlling GREEN ground on LED light strip
Aux2 -> Control signal to relay controlling BLUE ground on LED light strip

I have a PC 12V power supply currently supplying the power but will be moving to the 12V line in the Cisco PSU.


AMAZON LINKS:
-------------

I am currently using this relay:
https://www.amazon.ca/SunFounder-Channel-Arduino-Raspberry-Pi520141421293SunFounder/dp/B00E0NSORY/

I am currently using lights similar to these (make sure they are 12V when ordering):
https://www.amazon.ca/gp/product/B07TD8SJQT/