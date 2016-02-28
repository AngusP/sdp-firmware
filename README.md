Firmware User Guide
===================

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
available under Tools $>$ Serial Port $>$ /dev/ttyACM\[n\]. Make sure
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
garbage data. Details on how to configure the RF modules are left to
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

The command set that the robot currently recognises is documented in the
Technical Specification.


Firmware Technical Spec
=======================

Important Source Files
----------------------

#### `firmware.ino`

This is the main file which contains the `void setup()` and
`void loop()` functions. The former only initializes the serial port,
calls the `setup()` methods of other pivotal objects, and outputs
`STARTUP` to indicate that the robot is ready to receive commands. On
the other hand, the `loop` function handles asynchronous logic by
checking the serial port for any new commands and synchronous logic by
running processes (more on these below).

#### `SerialCommand.cpp`

This file provides a simple library whose purpose is to map command
strings to callbacks which it then executes based on the input it reads
from the serial port.

#### `CommandSet.cpp`

This object initializes `SerialCommand` and associates each of the
robot’s commands with one of its methods, which then perform the
necessary steps to fulfil it.

#### `Processes.cpp`

The object in this file manages processes, which are essentially tasks
to be completed periodically but not necessarily as often as the
`loop()` function is called.

#### `State.cpp`

This is a very simple object which essentially only serves as a central
storage space for various values which may change during runtime. Most
notably, it contains the current information on each of the holonomic
motors such as power or total distance travelled, and the state of a
data transfer.

#### `robot.cpp`

Contains the code that is run clock-synchronously by Processes – There
is a process that is responsible for toggling an LED every second in
order to indicate the robot is *alive*. Other processes include one that
polls the encoders on each motor to monitor how far they travelled and
another one that potentially makes any adjustments necessary to account
for errors or motor differences based on this information.

#### `SDPArduino.cpp`

This is a library based on the last year’s winner team’s firmware which
declares several low-level utility functions providing convenient
interface for bidirectional motor movement and halting.

Architecture
------------

The firmware we’re currently running has changed significantly from the
source we started with (SDP Group 13 2015). While the concepts haven’t
changed much, the implementation is significantly different, and the
majority of changes have been made to generalise the control structures
and add powerful flexibility to the firmware’s capabilities.

The notion of ‘Processes’ has been introduced, allowing a structure to
define a task that must be run at a regular interval. Process structures
carry around all their own scheduling information, and a pointer to the
function that needs to be run; This information is mutable, allowing
state machine behaviour with variable yet defined intervals between
transitions to be employed. The firmware as it stands uses Processes to
poll the rotary encoders on the drive motors, and calculate the
instantaneous speeds of each wheel. A Process was also used to guarantee
accurate timing for the Milestone 1 communications task.

Similarly, the parsing and execution of commands received over serial is
handled by the ‘CommandSet’ and ‘SerialCommand’ classes, with function
pointers being registered in association with the keyword string at
setup time. These abstractions make it easy to add processes or new
commands, and isolate potentially buggy functions from each-other
(though any blocking IO operations will still break functionality)

Command Set
-----------

All commands should be sent with a trailing newline, `\n`

Ping

:   This is the simplest command the firmware responds to and should
    immediately receive a `pong` back.

    send: `ping`

    gets: `pong`

Toggle LED

:   Toggles the LED on/off. This will get overridden pretty quickly by
    the heartbeat blinking, but you can verify your connection this
    way too.

    send: `L\n`

    gets: nothing

Go

:   Give the robot a $(x,y,\omega)$ vector and a power in the range 0 -
    255, which it’ll use to calculate and execute the holonomic motion
    in that direction. $x$ is the lateral (right +ve) direction, $y$ is
    the forward and $\omega$ is the rotational speed
    (anti-clockwise +ve). Note that the bot will always execute the
    motion at the maximum speed possible, so the `rotate` command should
    be used for precise rotations. The bounds on the variables are
    undefined, as their relative magnitudes are used though the
    recommended range is $-1.0$ to $1.0$. `G 0 0 0 0` or `G` will stop
    the robot. If the fourth argument is omitted then it will default to
    255 (full power).

    send: `G %f1 %f2 %f3 %d4`

    gets: `A`

    debug: `Going`

Move

:   Tells the robot to apply power to the motors in the order 1 $>$ 2
    $>$ 3. Valid range is -255 to 255 inclusive. `M 0 0 0` or `M` will
    stop the robot. This provides lower level / more direct control of
    the motors, and in general ‘Go’ is preferable.

    send: `M %d1 %d2 %d3`

    gets: `A`

    debug: `Moving`

Force Stop

:   Force a stop. The robot will remove power from the motors
    immediately, without trying to do any clever \`quick stop\` by
    braking the motors. Generally a singular `M` or `G` will
    work better.

    send: `S`

    gets: `A`

    debug: `Force stopping`

Motor Speeds

:   Will print the instantaneous speeds of all three drive motors in
    bogounits (encoder stops per second) in the form:

    send: `speeds`

    gets: `0: n.nn 1: n.nn 2: n.nn`

Grabbing

:   Opens or closes the grabber, depending on the argument given

    send: `grab <0/1>`

    gets: `A`

    debug: `grabbing <0/1>\n done`

Rotating

:   Rotates at `d1` power (-255 to 255 inclusive) until the encoders
    have gone through `d2` stops (0+) on average. This can be used to
    get fairly precice rotation but stops isn’t degrees (though
    it’s related) and depends somewhat on battery voltage.

    send: `rotate %d1 %d2`

    gets: `A`

    debug: `rotating at %d1 to %d2 stops`

Help

:   Prints the commands that the robot is listening for; this is always
    correct as it is generated on-the-fly from the Command Set
    parsing class. It won’t print how many arguments it expects, but is
    useful for checking spelling or troubleshooting a `N` (NACK).

    The help command can also be used to verify that the command set is
    the one you’re expecting.

    send: `help`

    get: `Valid input commands: (some have arguments)`\
    (Each command will be printed on a new line, excluding the number
    of arguments)

NACK

:   If the command is not recognised the bot will reply with a NACK:

    `N - <unrecognised_command>`


