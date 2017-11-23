/* /////////////////// - OpticFC v1.0.0a - //////////////////////////////////////////////////////////////////////////////
 * By Opticulex; Release date: 14/10/2017                                                                              //
 *                                                                                                                     //
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR                                          //
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,                                            //
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE                                         //
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER                                              //
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,                                       //
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE                                       //
 * SOFTWARE.                                                                                                           //
 *                                                                                                                     //
 * This is a completely barebones flight controller for use on an Arduino UNO or equivalent. It requires The FS-iA6B   //
 * Reciever module (or any other RX that gives out stanard PWM signal) and a stanard MPU-6050 Gyro/Accel I2C Module.   //
 *                                                                                                                     //
 * All the rest of the parts can just be ones you'd just put in any quad.                                              //
 *                                                                                                                     //
 * NOTE: Some older ESC's (and ones for larger motors etc) use a 3-wire servo connector while newer ones (that have    //
 * protocols like One/Multi/D-Shot on them) tend to only have a 2-wire connector or just bare solder pads. These new   //
 * ESCs will work if you connect GND and Signal up (see your ESCS manual for a schematic). This code is designed to    //
 * work with the Racerstar RS20A an RS30A ESC's.                                                                       //
 *                                                                                                                     //
 * ======= ABOUT THE CODE =======                                                                                      //
 * This code does not use a PID controller in it for simplicity and keeps everything down to the basics. It instead    //
 * just corrects for the Gyro/Accel error, like the Proportional of a PID loop. A mix/hybrid of the Derivative and     //
 * the Integral is used as well but not to the level of a real PID controller would. Hack one in if you really want :) //
 *                                                                                                                     //
 */                                                                                                                    //
#include <Servo.h>                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variable Declaration                                                                                                //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const int FC_min_throttle = 1070;                                                                                      // Minimum throttle when armed (set between 1030-1100)
const int FC_max_arm = 1080;                                                                                           // Maximum throttle to arm at (increase if quad refuses arm)
const int FC_max_angle = 30;                                                                                           // Maximum accel angle to arm at (increase if quad refuses arm)
const int FC_max_stabAngle = 30;                                                                                       // Maximum angle of attack in stab mode
const int FC_stage1 = 2000;                                                                                            // Stage1 ESC calibration value (default 2000)
const int FC_stage2 = 1000;                                                                                            // Stage2 ESC calibration value (default 1000)
float FC_Accel[] = {0,0,0};                                                                                            // Accelerometer XYZ values
float FC_Gyro[] = {0,0,0};                                                                                             // Gyroscope XYZ values
int FC_RPTY_in[] = {1000,1500,1500,1500};                                                                              // Reciever RPTY input values
int FC_RPTY_out[] = {1000,1500,1500,1500};                                                                             // Reciever RPTY output values (unused)
int FC_AUX_in[] = {1000};                                                                                              // Reciever AUX input values (channel 6 is disabled)
int FC_PID[] = {60,40,30,60,40,30,60,40,30};                                                                           // PID values (unused)
int FC_RATE[] = {100,100,100};                                                                                         // PID rates (unused)
int FC_Motors[] = {1000,1000,1000,1000};                                                                               // Motor 1234 values for ESC control
int FC_MotorsMixR[] = {1000,1000,1000,1000};                                                                           // Motor Mix Roll 1234 temp values
int FC_MotorsMixP[] = {1000,1000,1000,1000};                                                                           // Motor Mix Pitch 1234 temp values
int FC_MotorsMixY[] = {1000,1000,1000,1000};                                                                           // Motor Mix Yaw 1234 temp values
bool FC_Arm = false;                                                                                                   // Is quad armed
bool FC_Stab = true;                                                                                                   // Is quad in STAB mode, if not AIR mode (unused)
bool FC_Fsaf = false;                                                                                                  // Is quad failsafed (unused)
bool FC_ArmB = false;                                                                                                  // Temp var for FC arm LED blink
int FC_ThrottlePercent = 0;                                                                                            // Percentage (1-100) of throttle value
int FC_Cycle = 0;                                                                                                      // Total FC cycles
int FC_ArmCycle = 0;                                                                                                   // Total FC cycles while armed
Servo FC_Motor_1; Servo FC_Motor_2; Servo FC_Motor_3; Servo FC_Motor_4;                                                // Setup Pins: for ESC control out (output PWM signal to ESCS)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Control Methods                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void FC_ReadRX() {                                                                                                     // RX CHECK - Full check (Roll, Pitch, Throttle, Yaw, AUX1)
  FC_RPTY_in[0]=pulseIn(2, HIGH); FC_RPTY_in[1]=pulseIn(4, HIGH);                                                      //
  FC_RPTY_in[2]=pulseIn(7, HIGH); FC_RPTY_in[3]=pulseIn(8, HIGH);                                                      //
  FC_AUX_in[0]=pulseIn(12, HIGH);                                                                                      //
  if (FC_Arm==true and FC_RPTY_in[2]<FC_min_throttle){ FC_RPTY_in[2]=FC_min_throttle; }                                // min_throttle checker
  else if (FC_Arm==false) { FC_Motors[0]=1000; FC_Motors[1]=1000; FC_Motors[2]=1000; FC_Motors[3]=1000;                // Set motor values to 1000 if arm = false
  FC_MotorSet(); } }                                                                                                   //
void FC_ReadRX_lite() {                                                                                                // RX CHECK - Lite check (only check Roll, Pitch, Throttle)
  FC_RPTY_in[0]=pulseIn(2, HIGH); FC_RPTY_in[1]=pulseIn(4, HIGH);                                                      //
  FC_RPTY_in[2]=pulseIn(7, HIGH);                                                                                      //
  if (FC_Arm==true and FC_RPTY_in[2]<FC_min_throttle){ FC_RPTY_in[2]=FC_min_throttle; }                                // min_throttle checker
  else if (FC_Arm==false){                                                                                             //
    FC_Motors[0]=1000; FC_Motors[1]=1000; FC_Motors[2]=1000; FC_Motors[3]=1000; FC_MotorSet(); FC_MotorSet();          // Set motor values to 1000 if arm = false
    }                                                                                                                  //
  }                                                                                                                    //
void FC_ReadRX_aux() {                                                                                                 // RX CHECK - Arm check (only check AUX1, i.e. if quad is armed)
  FC_AUX_in[0]=pulseIn(12, HIGH); }                                                                                    //
void FC_ReadGyro(){                                                                                                    // Read gyro values
                                                                                                                       //
}                                                                                                                      //
void FC_ReadAccel(){                                                                                                   // Read accel values
                                                                                                                       //
}                                                                                                                      //
void FC_CrrctGyro(){                                                                                                   // Correct for gyro error
                                                                                                                       //
}                                                                                                                      //
void FC_CrrctAccel(){                                                                                                  // Correct for accel error
                                                                                                                       //
}                                                                                                                      //
void FC_MotorMix(){                                                                                                    // Mix raw channel values for each motor
  FC_Motors[0]=FC_RPTY_in[2]; FC_Motors[1]=FC_RPTY_in[2];                                                              // = Set all motor values to throttle value
  FC_Motors[2]=FC_RPTY_in[2]; FC_Motors[3]=FC_RPTY_in[2];                                                              // =
  if (FC_RPTY_in[0]<=1500){                                                                                            // ==== ROLL
    FC_ThrottlePercent=map(FC_RPTY_in[2], 1000, 2000, 0, 100);                                                         // Convert throttle 1000-2000 into percentage 0-100
    FC_Motors[2]=FC_Motors[2]+((FC_RPTY_in[0]-1500)*FC_ThrottlePercent/100);                                           // If roll is <= 1500 reduce motors 4/3 and
    FC_Motors[0]=FC_Motors[0]-((FC_RPTY_in[0]-1500)*FC_ThrottlePercent/100);                                           // speed up motors 2/1 by the difference from 1500
    FC_Motors[3]=FC_Motors[3]+((FC_RPTY_in[0]-1500)*FC_ThrottlePercent/100);                                           // and the current value, divided by the current
    FC_Motors[1]=FC_Motors[1]-((FC_RPTY_in[0]-1500)*FC_ThrottlePercent/100);                                           // throttle percent.
  }                                                                                                                    //
  else if (FC_RPTY_in[0]>1500){                                                                                        //
    FC_ThrottlePercent=map(FC_RPTY_in[2], 1000, 2000, 100, 0);                                                         //
    FC_Motors[2]=FC_Motors[2]-((1500-FC_RPTY_in[0])*FC_ThrottlePercent/100);                                           // If roll is > 1500 speed up motors 4/3 and
    FC_Motors[0]=FC_Motors[0]+((1500-FC_RPTY_in[0])*FC_ThrottlePercent/100);                                           // reduce motors 2/1 by the difference from 1500
    FC_Motors[3]=FC_Motors[3]-((1500-FC_RPTY_in[0])*FC_ThrottlePercent/100);                                           // and the current value, divided by the current
    FC_Motors[1]=FC_Motors[1]+((1500-FC_RPTY_in[0])*FC_ThrottlePercent/100);                                           // throttle percent.
  } FC_MotorCorrect();                                                                                                 // Correct motor values if they aren't between 1000-2000
  FC_MotorsMixR[0]=FC_Motors[0]; FC_MotorsMixR[1]=FC_Motors[1];                                                        // = Store modified motor values to Roll temp var
  FC_MotorsMixR[2]=FC_Motors[2]; FC_MotorsMixR[3]=FC_Motors[3];                                                        // =
  FC_Motors[0]=FC_RPTY_in[2]; FC_Motors[1]=FC_RPTY_in[2];                                                              // = Set all motor values to throttle value
  FC_Motors[2]=FC_RPTY_in[2]; FC_Motors[3]=FC_RPTY_in[2];                                                              // =
  if (FC_RPTY_in[1]<=1500){                                                                                            // ==== PITCH
    FC_ThrottlePercent=map(FC_RPTY_in[2], 1000, 2000, 0, 100);                                                         // Convert throttle 1000-2000 into percentage 0-100
    FC_Motors[2]=FC_Motors[2]+((FC_RPTY_in[1]-1500)*FC_ThrottlePercent/100);                                           // If pitch is <= 1500 reduce motors 3/1 and
    FC_Motors[0]=FC_Motors[0]+((FC_RPTY_in[1]-1500)*FC_ThrottlePercent/100);                                           // speed up motors 4/2 by the difference from 1500
    FC_Motors[3]=FC_Motors[3]-((FC_RPTY_in[1]-1500)*FC_ThrottlePercent/100);                                           // and the current value, divided by the current
    FC_Motors[1]=FC_Motors[1]-((FC_RPTY_in[1]-1500)*FC_ThrottlePercent/100);                                           // throttle percent.
  }                                                                                                                    //
  else if (FC_RPTY_in[1]>1500){                                                                                        //
    FC_ThrottlePercent=map(FC_RPTY_in[2], 1000, 2000, 100, 0);                                                         // Convert throttle 1000-2000 into percentage 0-100
    FC_Motors[2]=FC_Motors[2]-((1500-FC_RPTY_in[1])*FC_ThrottlePercent/100);                                           // If pitch is > 1500 reduce motors 4/2 and
    FC_Motors[0]=FC_Motors[0]-((1500-FC_RPTY_in[1])*FC_ThrottlePercent/100);                                           // speed up motors 3/1 by the difference from 1500
    FC_Motors[3]=FC_Motors[3]+((1500-FC_RPTY_in[1])*FC_ThrottlePercent/100);                                           // and the current value, divided by the current
    FC_Motors[1]=FC_Motors[1]+((1500-FC_RPTY_in[1])*FC_ThrottlePercent/100);                                           // throttle percent.
  } FC_MotorCorrect();                                                                                                 //
  FC_MotorsMixP[0]=FC_Motors[0]; FC_MotorsMixP[1]=FC_Motors[1];                                                        // = Store modified motor values to Pitch temp var
  FC_MotorsMixP[2]=FC_Motors[2]; FC_MotorsMixP[3]=FC_Motors[3];                                                        // =
  FC_Motors[0]=FC_RPTY_in[2]; FC_Motors[1]=FC_RPTY_in[2];                                                              // = Set all motor values to throttle value
  FC_Motors[2]=FC_RPTY_in[2]; FC_Motors[3]=FC_RPTY_in[2];                                                              // =
  if (FC_RPTY_in[3]<=1500){                                                                                            // ==== YAW
    FC_ThrottlePercent=map(FC_RPTY_in[2], 1000, 2000, 0, 100);                                                         // Convert throttle 1000-2000 into percentage 0-100
    FC_Motors[2]=FC_Motors[2]-((FC_RPTY_in[3]-1500)*FC_ThrottlePercent/100);                                           // If yaw is <= 1500 reduce CW motors and
    FC_Motors[0]=FC_Motors[0]+((FC_RPTY_in[3]-1500)*FC_ThrottlePercent/100);                                           // speed up CCW motors by the difference from 1500
    FC_Motors[3]=FC_Motors[3]+((FC_RPTY_in[3]-1500)*FC_ThrottlePercent/100);                                           // and the current value, divided by the current
    FC_Motors[1]=FC_Motors[1]-((FC_RPTY_in[3]-1500)*FC_ThrottlePercent/100);                                           // throttle percent.
  }                                                                                                                    //
  else if (FC_RPTY_in[3]>1500){                                                                                        //
    FC_ThrottlePercent=map(FC_RPTY_in[2], 1000, 2000, 100, 0);                                                         // Convert throttle 1000-2000 into percentage 0-100
    FC_Motors[2]=FC_Motors[2]+((1500-FC_RPTY_in[3])*FC_ThrottlePercent/100);                                           // If yaw is > 1500 reduce CCW motors and
    FC_Motors[0]=FC_Motors[0]-((1500-FC_RPTY_in[3])*FC_ThrottlePercent/100);                                           // speed up CW motors by the difference from 1500
    FC_Motors[3]=FC_Motors[3]-((1500-FC_RPTY_in[3])*FC_ThrottlePercent/100);                                           // and the current value, divided by the current
    FC_Motors[1]=FC_Motors[1]+((1500-FC_RPTY_in[3])*FC_ThrottlePercent/100);                                           // throttle percent.
  } FC_MotorCorrect();                                                                                                 //
  FC_MotorsMixY[0]=FC_Motors[0]; FC_MotorsMixY[1]=FC_Motors[1];                                                        // = Store modified motor values to Yaw temp var
  FC_MotorsMixY[2]=FC_Motors[2]; FC_MotorsMixY[3]=FC_Motors[3];                                                        // =
  FC_Motors[0]=(FC_MotorsMixR[0])+(FC_MotorsMixP[0])+(FC_MotorsMixY[0]);                                               // = Set motor values to the sum of motor RPTY temps added
  FC_Motors[1]=(FC_MotorsMixR[1])+(FC_MotorsMixP[1])+(FC_MotorsMixY[1]);                                               // = together and then divide by 3 to get the mean (mix) of
  FC_Motors[2]=(FC_MotorsMixR[2])+(FC_MotorsMixP[2])+(FC_MotorsMixY[2]);                                               // = Roll, Pitch, Throttle and Yaw.
  FC_Motors[3]=(FC_MotorsMixR[3])+(FC_MotorsMixP[3])+(FC_MotorsMixY[3]);                                               // =
  FC_Motors[0]=FC_Motors[0]/3; FC_Motors[1]=FC_Motors[1]/3;                                                            // = Yes, I know this system is sketchy, but it works.
  FC_Motors[2]=FC_Motors[2]/3; FC_Motors[3]=FC_Motors[3]/3;                                                            // =
  FC_MotorCorrect();                                                                                                   // Final motor value correction
}                                                                                                                      //
void FC_MotorCorrect(){                                                                                                // Correct motor values if incorrect (not between 1000-2000)
  if (FC_Motors[0]>2000) { FC_Motors[0]=2000; } if (FC_Motors[0]<1000) { FC_Motors[0]=1000; }                          //
  if (FC_Motors[1]>2000) { FC_Motors[1]=2000; } if (FC_Motors[1]<1000) { FC_Motors[1]=1000; }                          //
  if (FC_Motors[2]>2000) { FC_Motors[2]=2000; } if (FC_Motors[2]<1000) { FC_Motors[2]=1000; }                          //
  if (FC_Motors[3]>2000) { FC_Motors[3]=2000; } if (FC_Motors[3]<1000) { FC_Motors[3]=1000; }                          //
}                                                                                                                      //
void FC_MotorSet(){                                                                                                    // Apply motor values to ESC's
  FC_Motor_1.writeMicroseconds(FC_Motors[0]);                                                                          //
  FC_Motor_2.writeMicroseconds(FC_Motors[1]);                                                                          //
  FC_Motor_3.writeMicroseconds(FC_Motors[2]);                                                                          //
  FC_Motor_4.writeMicroseconds(FC_Motors[3]);                                                                          //
}                                                                                                                      //
void FC_RXinCorrect(){                                                                                                 // Correct RX in values if invalid (not between 1000-2000)
  if(FC_RPTY_in[0]>2000){ FC_RPTY_in[0]=2000; } if(FC_RPTY_in[0]<1000){ FC_RPTY_in[0]=1000; }                          //
  if(FC_RPTY_in[1]>2000){ FC_RPTY_in[1]=2000; } if(FC_RPTY_in[1]<1000){ FC_RPTY_in[1]=1000; }                          //
  if(FC_RPTY_in[2]>2000){ FC_RPTY_in[2]=2000; } if(FC_RPTY_in[2]<1000){ FC_RPTY_in[2]=1000; }                          //
  if(FC_RPTY_in[3]>2000){ FC_RPTY_in[3]=2000; } if(FC_RPTY_in[3]<1000){ FC_RPTY_in[3]=1000; }                          //
  if(FC_AUX_in[0]>2000){ FC_AUX_in[0]=2000; } if(FC_AUX_in[0]<1000){ FC_AUX_in[0]=1000; } }                            //
void FC_LED1_Blink(int FC_LED1_Blink_loops, int FC_LED1_Blink_delay){                                                  // LED Indicator control
  int FC_LED1_Blink_l=0;                                                                                               //
  while (FC_LED1_Blink_l<FC_LED1_Blink_loops){                                                                         //
    digitalWrite(5, HIGH); delay(FC_LED1_Blink_delay);                                                                 //
    digitalWrite(5, LOW); delay(FC_LED1_Blink_delay); ++FC_LED1_Blink_l;                                               //
  } FC_LED1_Blink_l=0; }                                                                                               //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Startup Setup                                                                                                       //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){                                                                                                          // STARTUP
  pinMode(5, OUTPUT);                                                                                                  // Setup Pins: for LED indicators
  FC_LED1_Blink(3,50);                                                                                                 // LED Indicator: blink 'FC startup' sequence
  pinMode(2, INPUT); pinMode(4, INPUT); pinMode(7, INPUT); pinMode(8, INPUT); pinMode(12, INPUT);                      // Setup Pins: for RX in (recieves PWM reciever input)
  FC_Motor_1.attach(11); FC_Motor_2.attach(10); FC_Motor_3.attach(9); FC_Motor_4.attach(6);                            // Setup Pins: for Servo Motor/ESC control
  FC_ReadRX();                                                                                                         // First RX read/RX correction
  delay(1000);                                                                                                         //
  while (FC_Cycle==0 and FC_RPTY_in[0]==0){                                                                            // No RX signal
    FC_LED1_Blink(1,100); delay(1000); FC_ReadRX();                                                                    // =- LED Indicator: blink 'FC Startup no RX error' sequence
  } FC_RXinCorrect();                                                                                                  //
  while (FC_Cycle==0 and FC_AUX_in[0]>=1500 and FC_RPTY_in[1]>=1870){                                                  // Go into ESC calib mode if armed and pitch full forward
    FC_LED1_Blink(3,100); delay(1000);                                                                                 // LED Indicator: blink 'ESC calibrate' sequence
    FC_Motors[0]=FC_stage1; FC_Motors[1]=FC_stage1; FC_Motors[2]=FC_stage1; FC_Motors[3]=FC_stage1; FC_MotorSet();     //
    FC_LED1_Blink(20,40); delay(10000);                                                                                //
    FC_Motors[0]=FC_stage2; FC_Motors[1]=FC_stage2; FC_Motors[2]=FC_stage2; FC_Motors[3]=FC_stage2; FC_MotorSet();     //
    FC_LED1_Blink(20,40); delay(10000); FC_ReadRX();                                                                   //
  }                                                                                                                    //
  while (FC_Cycle==0 and FC_AUX_in[0]>=1500){                                                                          // Pause startup is quad is in armed state
    FC_LED1_Blink(2,100); delay(1000); FC_ReadRX();                                                                    // =- LED Indicator: blink 'FC Startup arm error' sequence
  }                                                                                                                    //
  delay(500); FC_LED1_Blink(1,500);                                                                                    // LED Indicator: blink 'FC ready' sequence
}                                                                                                                      //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main Loop                                                                                                           //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){                                                                                                           // MAIN LOOP
  //                                                                                                                   //
  //                                                                                                                   //
  FC_ReadRX(); FC_RXinCorrect(); ++FC_Cycle;                                                                           // Read RX/Correct RX signal and increment FC cycle counter
  if (FC_AUX_in[0]>=1500){                                                                                             // If AUX1 (Arm channel) is high, start arming process
    FC_Arm=true;                                                                                                       // =
    if (FC_ArmB==false){                                                                                               // = LED Indicator: blink 'Arming' sequence
      FC_ArmB=true; FC_LED1_Blink(1,100);                                                                              // =
    }                                                                                                                  //
    while (FC_ArmCycle==1 and FC_RPTY_in[2]>=FC_max_arm){                                                              // = Refuse arm when throttle is above 'max_arm'
      FC_LED1_Blink(1,100); FC_ReadRX_lite();                                                                          // = LED Indicator: blink 'Error' sequence
    }                                                                                                                  //
  } else {                                                                                                             // = If Arm channel is low just make sure 'Arm' is false
    FC_Arm=false; FC_ArmB=false;                                                                                       // =
  }                                                                                                                    //
  if (FC_Arm==true){                                                                                                   // If armed
    ++FC_ArmCycle;                                                                                                     // Increment FC arm cycle counter
    FC_MotorMix();                                                                                                     // Mix channel values to work with motors
    FC_ReadGyro(); FC_CrrctGyro();                                                                                     // Read/Correct for gyro
    FC_ReadAccel(); FC_CrrctAccel();                                                                                   // Read/Correct for accel
    FC_MotorSet();                                                                                                     // Push motor values to ESC's as a signal
  } else {                                                                                                             //
    FC_ArmCycle=0;                                                                                                     // If not armed reset FC arm cycle counter
  }                                                                                                                    //
}                                                                                                                      //
