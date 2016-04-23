Userguide
=========

Compiling & Uploading
---------------------

All the necessary source files to compile and run the firmware on an
Arduino Uno or compatible microcontroller are in the root directory of
the firmware Git repository. The code has been tested on both an Arduino
Uno and a Xino RF, both of which are designed around the Atmel ATMega
32u4 microcontroller. The code is not guaranteed to work on any other
boards though it should be fine in most cases.

To compile, open `firmware.ino` in the Arduino IDE. Note that the IDE
must be version 1.6.0 or greater; The version currently bundled with
DICE is too old to support some of the C++ 11 features used in this
code. We will also assume that you are using a Linux based system. Also
note that the IDE’s automagic linker is fussy, and will want
`firmware.ino`’s parent directory to also be called `firmware`. The most
recent release of the Arduino IDE is available from
<https://arduino.cc>.

All dependencies are either included with the repository or come bundled
with the Arduino IDE.

Ensure the Arduino you wish to program is plugged in, it’s power light
is on (if it has one) and the IDE can detect it - if it can, it’ll be
available under Tools $\rightarrow$ Serial Port $\rightarrow$
/dev/ttyACM\[n\]. Arduino’s website and forums have good documentation
for dealing with drivers and connection issues. Make sure that the
correct board is selected - ‘Arduino/Genuino Uno’ works for both the Uno
and Xino. To upload, hit the upload button (the arrow in the blue
toolbar) which will proceed to flash the board. See
[https://arduino.cc](https://www.arduino.cc/en/Guide/Troubleshooting)
for troubleshooting uploads.

The only visible confirmation that the firmware is running correctly is
that the status LED on the board should be blinking on and off every
second. The firmware will begin running as soon as the board is powered
on, and print `STARTUP` over the serial port as soon as it is ready to
receive a command.

Serial Port Setup
-----------------

The firmware communicates at `115200` Baud, meaning that the serial
terminal you’re using, the RF module on the Xino, and the RF USB modem
stick must all be using this Baud rate, otherwise you will receive
garbage data. Details on how to configure the RF modules are left to
Xino’s own user guide, to save repeating information.

You’ll need a Serial Console with a ‘line mode’.
[‘CoolTerm’](http://freeware.the-meiers.org/) is a capable free tool,
though PuTTY also has a ‘Line Editing Mode’ that is perfectly useable in
conjunction with ‘Local Echo’. The Arduino IDE’s built in Serial Monitor
is simple but will also work. Make sure it is communicating at the
correct Baud rate, and is set to send a ‘newline’ after the string is
sent to the Arduino.

The command set that the robot currently recognises is documented in the
Technical Specification. No further setup is required on the firmware
side.


Tech Spec
=========

Architecture
------------

The firmware we’re currently running has changed significantly from the
source we started with (SDP Group 13 2015). Some useful concepts have
survived, but the implementation is significantly different, and the
majority of changes have been made to generalise the control structures
and add powerful flexibility to the firmware’s capabilities.

The notion of ‘Processes’ has been introduced, allowing a structure to
define a task that must be run at a defined time interval. Process
structures carry around all their own scheduling information, and a
pointer to the function that needs to be run; This information is
mutable, allowing temporal state machine behaviour with time-triggered
transitions to be employed. The firmware as it stands uses Processes to
poll the rotary encoders on the drive motors, and calculate the
instantaneous speeds of each wheel. A Process was also used to guarantee
accurate timing for the Milestone 1 communications task. This
architecture adds minimal overhead but verifiably gives an abstracted
way of serially multitasking, as opposed to a very large loop function
that risks control blocking and lock-ups.

Similarly, the parsing and execution of commands received over serial is
handled by the ‘CommandSet’ class and ‘SerialCommand’ subclass, with
function pointers being registered in association with the keyword
string at setup time. Commands received over Serial are interpreted and
executed immediately in an interrupt-driven model, giving very little
lag between a command being sent and a command being enacted by the bot.
These abstractions make it easy to add processes or new commands, and
isolate potentially buggy functions from each-other (though any blocking
IO operations will still break functionality). It is also possible to
enable and disable processes during runtime, if it is desired.

Motion Execution and Correction
-------------------------------

A process (as defined above) is used to read the changes in rotary
encoder positions from the three NXT motors. Another process then uses
this data to correct to motor speeds to reflect the motion the robot was
commanded to execute. The correction process uses Gradient Descent,
which is necessary due to the alinear nature of the Motors’ rotational
speed with respect to power applied, coupled with the fact that each of
the three motors has a different signature response.

The error vector $\hat{e}$ given a desired velocity vector $\hat{v}$ and
realised velocity vector $\hat{r}$ is defined:

$$\hat{e}_i = k \cdot \left(\frac{\hat{d}_i}{\Vert\hat{d}\Vert} - \frac{\hat{r}_i}{\Vert\hat{r}\Vert}\right)$$

The new powers we need to apply to the motors to reduce the error,
$\hat{c'}$ with respect to the previous $\hat{c}$ and error $\hat{e}$ is

$$\hat{c}_i' = \frac{ \hat{c}_i }{\Vert\hat{c}\Vert} + \frac{ \hat{e}_i }{\Vert\hat{e}\Vert}$$

The error function we wish to minimise is $E$

$$E_{(\hat{d},\hat{r})} = \sqrt{\sum_{j=1}^{n} (\hat{d_j} - \hat{r_j})^2}$$

Which has the partial derivative with respect to $\hat{r}_i$

$$\frac{\partial E}{\partial \hat{r}_i}
  =
  \frac{\hat{r}_i - \hat{d}_i}{\sqrt{\sum_{j=1}^{n} (\hat{d_j} - \hat{r_j})^2}}
  =
  \frac{\hat{r}_i - \hat{d}_i}{\Vert\hat{d}-\hat{r}\Vert}$$

This component is crucial to the accuracy and repeatability of motion
for the robot.

As far as units go, all vectors are converted to unit vectors, but most
are provided initially with dimensions in the range $[-255,255]$ with
the exception of the realised velocity vector, which is simply the
number of ticks the rotary encoders have passed in the time interval
since last checked. The conversion to unit vectors is in part to
eliminate problems with unit compatibility, and part because the
single-precision only floating point unit on the Arduino quickly
succumbs to NaNs if dealing with larger numbers.
