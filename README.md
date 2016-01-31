Firmware
========

Compiling & Uploading
---------------------

All the necessary source files to compile and run the firmware on an
Arduino Uno or compatible microcontroller are in the root directory of
the firmware Git repository. The code has been tested on both an Arduino
Uno and a Xino RF, both of which are designed around the Atmel AT328P
microcontroller. The code is not guaranteed to work on any other boards
though it will be fine in most cases.

The main source file is firmware.ino, from which the other source files
are included. To compile, it is easiest to open `firmware.ino` in the
Arduino IDE. Note that the IDE must be version 1.6.0 or greater; The
version currently bundled with DICE is too old to support some of the
C++ 11 features used in this code. We will also assume that you are
using a Linux based system. Also note that the IDE’s automagic linker is
fussy, and will want `firmware.ino`’s parent directory to also be called
`firmware`. The most recent build of the Arduino IDE is available from
<http://arduino.cc>.

All dependencies are either included with the repository or come bundled
with the Arduino IDE.

Ensure the Arduino you wish to program is plugged in, it’s power light
is on (if it has one) and the IDE can detect it – if it can, it’ll be
available under Tools  \>  Serial Port  \>  /dev/ttyACM[n]. Make sure
that the correct board is selected – ‘Arduino/Genuino Uno’ works for
both the Uno and Xino. To upload, hit the upload button (the arrow in
the blue toolbar) which will proceed to flash the board. See
<http://arduino.cc> for troubleshooting uploads.

The only visible confirmation that the firmware is running correctly is
that the status LED on the board should be blinking on and off every
second. The firmware will begin running as soon as the board is powered
on, and print `STARTUP` over the serial port as soon as it is ready to
receive a command.

Serial Port Setup
-----------------

Some of the specifics will be different depending on which Arduino board
is used; This guide assumes the use of the Xino RF. Using Serial over
USB is pretty straightforward so is useful as a sanity check if the RF
is giving you trouble.

The firmware communicates at `115200` Baud, meaning that the serial
terminal you’re using, the RF module on the Xino, and the RF USB modem
stick must all be using this Baud rate, otherwise you will only receive
garbage data. I’ll leave details on how to configure the RF modules to
Xino’s own user guide, to save repeating information and perhaps making
a mistake in doing so.

Unfortunately a decent serial terminal is hard to come by on Linux; We
need one with a ‘line mode’. Minicom can be coerced into doing this by
sending an ASCII text file to the device, but this is less than ideal. I
find that [‘CoolTerm’](http://freeware.the-meiers.org/) is the most
capable free tool available, though PuTTY also has a ‘Line Editing Mode’
that is perfectly useable in conjunction with ‘Local Echo’. The Arduino
IDE’s built in Serial Monitor is simple but will also work. Make sure it
is communicating at the correct Baud rate, and is set to send a
‘newline’ after the string is sent to the Arduino.

Command Set
-----------

`ping\n`  
This is the simplest command the firmware responds to and should
immediately receive a `pong` back.

`L\n`  
Toggles the LED on/off. This will get overridden pretty quickly by the
heartbeat blinking, but you can verify your connection this way too.

`M %d1 %d2 %d3\n`  
Tells the robot to apply power to the motors in the order 1  \>  2  \> 
3. Valid range is -255 to 255 inclusive. `M 0 0 0\n` will stop the
robot.

Gets the reply `Moving\n 0: d1 1: d2 2: d3`

`S\n`  
Force a stop. The robot will remove power from the motors immediately,
without trying to do any clever \`quick stop\` by braking the motors.

Will get the reply `Force stopping`

`Speeds\n`  
Will print the instantaneous speeds of all three drive motors in
bogounits (encoder stops per second) in the form:

`0: n.nn 1: n.nn 2: n.nn`

`kick %d1 %d2\n`  
Kicks with power `d1` (motor power, -255 to 255 inclusive) for `d2`
milliseconds.

`rotate %d1 %d2\n`  
Rotates at `d1` power (-255 to 255 inclusive) until the encoders have
gone through `d2` stops (0+) on average. This can be used to get fairly
precice rotation but will need calibration and depends on battery life &
power as the bot currently overrotates somewhat.

`help\n`  
Prints the commands that the robot is listening for; this is always
correct as it is generated on-the-fly from the Command Set parsing
class. It won’t print how many arguments it expects, but is useful for
checking spelling or troubleshooting a `N` (NACK).

Expected output:

    Valid input commands: (some have arguments)
    L\n ping\n help\n M\n S\n Speeds\n kick\n rotate
