# OpticFC



OpticFC is a barebones Flight Controller designed for the Arduino Uno and use for Miniquads.




Currently this version does not have a Gyro or Accel integrated into it for technical reasons but this is under development and should be included in the coming weeks. It isn't recommended you use it in its current state due to this.

## How to use

The controller uses an Arduino Uno R3 (clone compatible), FS-iA6B Reciever and MPU-6050 I2C Gyro/Accel module as well as an LED, some DuPont wires and a mini breadboard. It is recommended you use a Flysky FS-i6 radio for compatibility and ease of setup. Make sure your radio has PPM output turned off (so the FS-iA6B reciever is giving out individual-channel PWM output) and Channel 5 is configured as your arm switch (2 pos switch) and Channel 6 is configured as a 3 pos switch.

### How to wire up
The wiring for the system uses [this schematic](https://github.com/Opticulex/OpticFC/blob/master/OpticFC_Schematics/OpticFC_noGyroAccel.png). Unlike how you may setup a normal off-the shelf Flight controller, you **must** connect both the ESC Signal and ground up to the Arduino.

### How to calibrate ESCs

**DO NOT arm your quad before you calibrate your ESC's using the Arduino! Also, MAKE SURE that your ESC's actually can be calibrated with the max-min throttle method or it will end up in your ceiling. You have been warned!**

Turn on your Arduino with your radio armed and pitch set to max. Wait for the LED to indicate ESC calibration (see below) and then let go of the sticks and disarm. It will calibrate all ESCs (you should hear ESC calibration tones) and then resume normal startup. This takes about 20 seconds. It is higly recommended you do this without props on to avoid injury.

### Channels

The flight controller is designed to run off a 6-channel radio even though only 5 channels are used. Below is a breif description of what each channel does, for configuration:

**Channel 1,2,3,4:**

Roll, Pitch, Throttle, Yaw

**Channel 5:** 

>1500 - Disarmed and idle
<1500 - Armed, motors spinning at min_throttle

**Channel 6:** 

Channel 6 is not defaultly configured to anything but is used as a mode switch between FST mode (less input delay, less precision) and SMT mode (more input delay, smoother flight) on some experimental versions and also functions as a 'signal lost indicator'. None of these features are good enough to be production-ready though.


## LED Indicator

The system uses its LED indicator to display its current status and give feedback.

Normal startup consists of 3 fast blinks to signal startup and normal calibration followed by one long blink to signal the flight controller ready and idle. If, inbetween these two sets of blinks there is any other repeated blink sequence, it could signal an error code or a different startup mode beign entered:

1 blink - No RX signal found (not in use)

2 blinks - Error: Disarm quad

3 blinks - ESC calibration mode active

4 blinks - Incorrect settings and variables

5 blinks - Internal error

If you recieve a 1-blink indicator when attempting to arm, bring down the throttle to as low as possible.


THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.