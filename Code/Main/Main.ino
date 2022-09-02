/**
 * 5 buttons, up down left right, and the knob has a click



up: increments an index, this index is assigned to a stepper motor which the knob can manually control (good for calibration)



down : inverse of up



left: resets and moves all motors to our 'maximum' (good for calibrating our maximum)



right: cycles through each pre-programmed shape



knob click: toggles activation of manual stepper control



knob rotary action: moves steps on 1 stepper by some multiplier 
  (you won't see any height change if you move 1 step at a time because there's a gearing system which means you need a lot of steps) 
    IIRC 6000 steps is roughly the entire range
 */

#include <Encoder.h>
#include <AccelStepper.h>
#include <Bounce2.h>
//#include "AS5311.h"
#include <elapsedMillis.h>


#define GRID_SIZE       4
#define STEPPER_1       2
#define STEPPER_2       4
#define STEPPER_3       1
#define STEPPER_4       0
#define STEPPER_5       9
#define STEPPER_6       8
#define STEPPER_7       7
#define STEPPER_8       6
#define STEPPER_9       27
#define STEPPER_10      26
#define STEPPER_11      25
#define STEPPER_12      24
#define STEPPER_13      32
#define STEPPER_14      30
#define STEPPER_15      29
#define STEPPER_16      28

#define NUM_OF_STEPPERS 16
#define PWM_FREQ        10000

#define STEPPER_SPEED   1000   //
#define STEPPER_ACC     3000   //
#define STEPPER_OFFSET  25    // a small amunt of steps to remove backlash 

#define STEPPER_I1      11
#define STEPPER_I2      10
#define STEPPER_MS1     5       //low low = full step; H L = 1/2, L H = 1/4
#define STEPPER_MS2     12
#define STEPPER_DIR     31
#define STEPPER_SLEEP   3       // L = disable board

//#define AS5311_CSn      40
//#define AS5311_CLK      39
//#define AS5311_DO       38
//#define AS5311_PWM      37
//#define AS5311_Index    36
//#define AS5311_A        41
//#define AS5311_B        35
//#define AS5311_MagDECn  15
//#define AS5311_MagNCn   14

#define ROTARY_ENC_A    33
#define ROTARY_ENC_B    34
#define ROTARY_ENC_COM  18
#define ROTARY_ENC_NO   19      // BUTTON
#define ROTARY_DIVIDER  4       // each click is 4 counts so we divide a read by 4

#define DEBUG_LED       13
#define DEBUG_BTN_LEFT  16
#define DEBUG_BTN_RIGHT 17
#define DEBUG_BTN_UP    22
#define DEBUG_BTN_DOWN  21


// shape variables
int shape_one[4][4] = {     \
  {1200, 1200, 1200, 1200}, \
  {800, 800, 800, 800}, \
  {400, 400, 400, 400}, \
  {0, 0, 0, 0}
};
int shape_two[4][4] = {     \
  {1200, 1200, 1200, 1200}, \
  {1200, 1200, 1200, 1200}, \
  {1200, 1200, 1200, 1200}, \
  {1200, 1200, 1200, 1200}
};
int shape_three[4][4] = {     \
  {0,0,0,0}, \
  {0,0,0,0}, \
  {0,0,0,0}, \
  {0,0,0,0}
};

int shape_num = -1; // this does NOT need to be global


//stepper variables
uint8_t stepper_array[NUM_OF_STEPPERS] = {
  STEPPER_1, STEPPER_2, STEPPER_3, STEPPER_4,     \
  STEPPER_5, STEPPER_6, STEPPER_7, STEPPER_8,     \
  STEPPER_9, STEPPER_10, STEPPER_11, STEPPER_12,  \
  STEPPER_13, STEPPER_14, STEPPER_15, STEPPER_16  \
};
bool isStepperEnabled = false;

AccelStepper stepper_1(AccelStepper::DRIVER, STEPPER_1, STEPPER_DIR);
AccelStepper stepper_2(AccelStepper::DRIVER, STEPPER_2, STEPPER_DIR);
AccelStepper stepper_3(AccelStepper::DRIVER, STEPPER_3, STEPPER_DIR);
AccelStepper stepper_4(AccelStepper::DRIVER, STEPPER_4, STEPPER_DIR);
AccelStepper stepper_5(AccelStepper::DRIVER, STEPPER_5, STEPPER_DIR);
AccelStepper stepper_6(AccelStepper::DRIVER, STEPPER_6, STEPPER_DIR);
AccelStepper stepper_7(AccelStepper::DRIVER, STEPPER_7, STEPPER_DIR);
AccelStepper stepper_8(AccelStepper::DRIVER, STEPPER_8, STEPPER_DIR);
AccelStepper stepper_9(AccelStepper::DRIVER, STEPPER_9, STEPPER_DIR);
AccelStepper stepper_10(AccelStepper::DRIVER, STEPPER_10, STEPPER_DIR);
AccelStepper stepper_11(AccelStepper::DRIVER, STEPPER_11, STEPPER_DIR);
AccelStepper stepper_12(AccelStepper::DRIVER, STEPPER_12, STEPPER_DIR);
AccelStepper stepper_13(AccelStepper::DRIVER, STEPPER_13, STEPPER_DIR);
AccelStepper stepper_14(AccelStepper::DRIVER, STEPPER_14, STEPPER_DIR);
AccelStepper stepper_15(AccelStepper::DRIVER, STEPPER_15, STEPPER_DIR);
AccelStepper stepper_16(AccelStepper::DRIVER, STEPPER_16, STEPPER_DIR);
AccelStepper* steppers[NUM_OF_STEPPERS] = {
  &stepper_1, &stepper_2, &stepper_3, &stepper_4,     \
  &stepper_5, &stepper_6, &stepper_7, &stepper_8,     \
  &stepper_9, &stepper_10, &stepper_11, &stepper_12,  \
  &stepper_13, &stepper_14, &stepper_15, &stepper_16  \
};
uint8_t stepper_index = 0;  // current stepper ?
int stepper_scaler = 50;     //multiplier


//serial variables
String inputString = "";         // a String to hold incoming data
int prevRead = 0;
bool stringComplete = false;  // whether the string is complete


//rotary variables
long oldPosition  = -999;
Encoder myEnc(ROTARY_ENC_A, ROTARY_ENC_B);


// button variables
Bounce left_but = Bounce();
Bounce right_but = Bounce();
Bounce up_but = Bounce();
Bounce down_but = Bounce();
Bounce rotary_but = Bounce();


//mag enc variables
//AS5311 as5311(38, 39, 40); // data, clock, chip select
//extern "C" uint32_t set_arm_clock(uint32_t frequency);
//elapsedMillis mag_enc_millis;
//const int mag_enc_delay = 25;
//int counter, zeroVal, finalPoint, encoderVal, lastEncoderVal;
//int lastPole = -1;
//float valueMap;
//float offsetValueMap = 0;

void setup() {
  init();
}

void loop() {
  button_handler();

  if (stringComplete) {
    inputString.trim();
    int in = inputString.toInt();
    Serial.println(in);
    inputString = "";
    stringComplete = false;
  }
  
  if (isStepperEnabled)
    rotary_handler();



}
