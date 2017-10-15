# OpticFC v1.0.0a



OpticFC is a barebones Flight Controller desined for the Arduino Uno and use for Miniquads.




Currently this version does not have a Gyro or Accel integrated into it for technical reasons but this should be included in the coming weeks.

## How to use

The controller uses an Arduino Uno R3, FS-iA6B Reciever and MPU-6050 I2C Gyro/Accel module as well as an LED, some DuPont wires and a mini breadboard. It uses [this schematic](https://github.com/Opticulex/OpticFC/blob/master/OpticFC_noGyroAccel.png). It is recommended you use a Flysky FS-i6 radio for compatibility. Make sure your radio has PPM output turned off and Channel 5 is configured as your arm switch.

### How to calibrate ESCs

Turn on your Aruino with your radio armed and pitch set to max. Wait for the LED to indicate ESC calibration (see below) and then let go of the sticks and disarm. It will calibrate all ESCs and then resume normal startup. This takes about 20 seconds. It is recommended you do this without props on.

## LED Indicator


The system uses its LED indicator to display its current status and give feedback.






Normal startup consists of 3 fast blinks to signal startup an calibration followed by one long blink to signal the flight controller ready. If, inbetween these two sets of blinks there is any other repeated blink sequence, it could signal an error code or a different startup mode beign entered:

1 blink - No RX signal found

2 blinks - Disarm quad

3 blinks - ESC calibration mode active

4 blinks - Incorrect settings and variables

5 blinks - Internal error



THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
