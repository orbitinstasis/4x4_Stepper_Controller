void init() {
  // serial setup
  Serial.begin(115200);
  inputString.reserve(200);  // hack: reserve 200 bytes for the inputString:


  for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
    steppers[i]->setMaxSpeed(STEPPER_SPEED);
    steppers[i]->setAcceleration(STEPPER_ACC);
  }
  // stepper driver setups
  analogWriteFrequency(STEPPER_I1, PWM_FREQ);
  //stepper driver pins setup
  pinMode(STEPPER_I1, OUTPUT);
  analogWrite(STEPPER_I1, 235);
  pinMode(STEPPER_I2, OUTPUT);
  digitalWrite(STEPPER_I2, LOW);
  pinMode(STEPPER_MS1, OUTPUT);
  digitalWrite(STEPPER_MS1, LOW);
  pinMode(STEPPER_MS2, OUTPUT);
  digitalWrite(STEPPER_MS2, LOW);
  pinMode(STEPPER_DIR, OUTPUT);
  digitalWrite(STEPPER_DIR, LOW);
  pinMode(STEPPER_SLEEP, OUTPUT);
  digitalWrite(STEPPER_SLEEP, LOW);

  //init the steppers to a known position
  reset_steppers();

  // rotary encoder setup [knob]
  pinMode(ROTARY_ENC_COM, OUTPUT);
  digitalWrite(ROTARY_ENC_COM, LOW);

  // buttons setup and debounce setup
  left_but.attach(DEBUG_BTN_LEFT, INPUT_PULLUP);
  right_but.attach(DEBUG_BTN_RIGHT, INPUT_PULLUP);
  up_but.attach(DEBUG_BTN_UP, INPUT_PULLUP);
  down_but.attach(DEBUG_BTN_DOWN, INPUT_PULLUP);
  rotary_but.attach(ROTARY_ENC_NO, INPUT_PULLUP);
  left_but.interval(25);
  right_but.interval(25);
  up_but.interval(25);
  down_but.interval(25);
  rotary_but.interval(25);
}

// MOVE ALL STEPPERS TO THE MAXIMUM POSITION
//if left button pushed, reset steppers and reinit to new max speed then move until hit max steps or max magEnc value
void findMax() {
  Serial.print("Debug: Moving steppers to max position...\n ");
  reset_steppers();
  stepper_driver_power(1);
  elapsedMillis loopDuration = 0;
  while (loopDuration < 2000) {    // move all the motors to 6000 and block for 5 seconds assuming it's all done by then
    for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
      //        fetch_mag_val();
      steppers[i]->moveTo(STEPPER_MAX);
      //      Serial.println(steppers[i]->currentPosition());
      //        if ((valueMap  - offsetValueMap>= 6) || (steppers[i]->currentPosition() >=4000)) { // second condition not meeting
      steppers[i]->run();
    }
  }
  for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) { // stop the steppers and reset the max speed
    steppers[i]->stop();
  }
  Serial.print("Moved to max.\n");
}

// go to the previous shape
void leftButton() {
  if (shape_num > 0)
    shape_num--;
  else
    shape_num = NUM_OF_SHAPES - 1;
  set_shape();
}

// go to the next shape
void rightButton() {
  if (shape_num < NUM_OF_SHAPES - 1)
    shape_num++;
  else
    shape_num = 0;
  set_shape();
}

// move the stepper index up so the rotary knob moves it manually, reset the old position
void upButton() {
  if (stepper_index < NUM_OF_STEPPERS - 1)
    stepper_index++;
  myEnc.write(0);
  //    oldPosition = 99999;
  Serial.print("Stepper index: "); Serial.println(stepper_index);

}

// move the stepper index down so the rotary knob moves it manually
void downButton() {
  if (stepper_index > 0)
    stepper_index--;
  myEnc.write(0);
  //    oldPosition = 99999;
  Serial.print("Stepper index: "); Serial.println(stepper_index);
}

//  toggles manual calibration activation for each stepper
void encoderButton() {
  isManualControl = !isManualControl;
  stepper_driver_power(isManualControl);
  Serial.print("Steppers enable: "); Serial.println(isStepperEnabled);
}

//poll button state
void button_handler() {
  left_but.update();
  right_but.update();
  up_but.update();
  down_but.update();
  rotary_but.update();
  if ( left_but.fell() ) {
    leftButton();
  }
  if ( right_but.fell() ) {
    rightButton();
  }
  if ( up_but.fell() ) {
    upButton();
  }
  if ( down_but.fell() ) {
    downButton();
  }
  if ( rotary_but.fell() ) {
    encoderButton();
  }
}


//rotary knob functinoality, moves a stepper manually
//call this if you have manual control turned on "isStepperEnabled = 1"
void rotary_handler() {
  //    fetch_mag_val();
  // move stepper_index manually with the knob
  long newPosition = (myEnc.read() / ROTARY_DIVIDER) * stepper_scaler;
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.print("Moving stepper index: "); Serial.print(stepper_index); Serial.print(" to: "); Serial.println(newPosition);
    //    steppers[stepper_index]->moveTo(newPosition);
    steppers[stepper_index]->runToNewPosition(newPosition);  // this should block until done
  }
  isStepperEnabled = false;
}

// reset Steppers:
void reset_steppers() {
  Serial.print("Resetting... ");
  isManualControl = 0;
  stepper_driver_power(1); // turn steppers on
  for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
    steppers[i]->moveTo(-4000);             // move all steppers to an obviuosly out of bounds extreme position so they all reset.
  }
  unsigned long time_now = millis();
  while (millis() - time_now <= 2000) {
    for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
      steppers[i]->run();                   // move the motors for a few secnds
    }
  }
  for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
    steppers[i]->setCurrentPosition(0);     //  re set the current position assingning it to 0
  }
  delay(200);
  for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
    steppers[i]->moveTo(STEPPER_OFFSET);    // from a known dead state, move the steppers a tiny bit in the opposite direction to kill any backlash
  }
  time_now = millis();
  while (millis() - time_now <= 200) {
    for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
      steppers[i]->run();                   // move the offset
    }
  }
  for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
    steppers[i]->setCurrentPosition(0);     //  re set the current position assingning it to 0
  }
  stepper_driver_power(0);                  // kill the power t the steppers
  //  fetch_mag_val();
  //  offsetValueMap = valueMap;
  delay(5);
  myEnc.write(0);
  count = 0;
  oldPosition = 99999;
  Serial.print("Reset.\n");
}







// input true or false, turns steppers on or off - you want them off when you're not moving them to reduce heat and increase lifespan
void stepper_driver_power(bool in) {
  isStepperEnabled = in;
  digitalWrite(STEPPER_SLEEP, isStepperEnabled);
  digitalWrite(DEBUG_LED, isStepperEnabled);
}

// moves the steppers to the predefined shapes
void set_shape() {
  Serial.print("Moving to shape: "); Serial.println(shape_num);
  int count = 0;
  for (uint8_t i = 0; i < 4; i++) {   // for each 2D cell, look at global shape num var and move all steppers sequentially to the assigned positions
    for (uint8_t j = 0; j < 4; j++) {
      steppers[count]->moveTo((*shapes[shape_num])[i][j]);; // not messing with sign as it dictates direction
      count++;
    }
  }
  stepper_driver_power(1); // turn the power on

  unsigned long time_now = millis();
  while (millis() - time_now <= 3000) {
    for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
      steppers[i]->run();                     // run to the positions
    }
  }
  //  for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
  //    steppers[i]->setCurrentPosition(0);       // delete if necessary, this resets the current position to 0 [i think this is wrong logic here so i deleted]
  //  }
  stepper_driver_power(0);            // turn the steppers off
  Serial.println("Moved to shape.");
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}


//void checkCounter() {
//
//  uint32_t triPole = as5311.triPole(encoderVal);
//
//  if (lastPole == 2 && triPole == 0) {
//    counter++;
//  }
//
//  if (lastPole == 0 && triPole == 2) {
//    counter--;
//  }
//
//  lastEncoderVal = encoderVal;
//  lastPole = triPole;
//}



//int dynamicZeroError() {
//  if (Serial.available() > 0) {
//    // read the incoming byte:
//    int readVal = Serial.read();
//
//    if (readVal == 48)
//    {
//      zeroVal = encoderVal;
//      finalPoint = 4095 - encoderVal;
//    }
//  }
//
//  return as5311.dynamic_zero(encoderVal, zeroVal, finalPoint);
//}



//void showData() {
//
//  Serial.print(encoderVal);
//  Serial.print(" ");
//
//  Serial.print(valueMap / 1000);
//  Serial.print("mm ");
//
//  if (as5311.error()) {
//
//    if (as5311.err_value.OCF) {
//      Serial.print(" OCF");
//    } else {
//      Serial.print("    ");
//    }
//    if (as5311.err_value.COF) {
//      Serial.print(" COF");
//    } else {
//      Serial.print("    ");
//    }
//    if (as5311.err_value.DECn) {
//      Serial.print(" DEC");
//    } else {
//      Serial.print("    ");
//    }
//    if (as5311.err_value.INCn) {
//      Serial.print(" INC");
//    } else {
//      Serial.print("    ");
//    }
//    if (as5311.err_value.LIN) {
//      Serial.print(" LIN");
//    } else {
//      Serial.print("    ");
//    }
//
//    Serial.println();
//  }
//}

//void fetch_mag_val() {
//  // if (mag_enc_millis >= mag_enc_delay) {
//    set_arm_clock( 24000000 );
//    //Step 1: Get the value
//    encoderVal = as5311.value();
//
//    //Step 2: Change direction
//    encoderVal = map(encoderVal, 0, 4095, 4095, 0);
//
//    //Step 3: Get the zero error and use dynamic zeroing
//    encoderVal = dynamicZeroError();
//
//    //Step 4: Increment or decrement counter variable
//    checkCounter();
//
//    //Step 5: Assign a proper value
//    valueMap = (((counter * 2000) + encoderVal * 0.488) / 1000);
//
//    //Show some data
//    //      showData();
//    as5311.error();
//    if (!as5311.err_value.DECn) { // if there's not a decn error
//      Serial.print(valueMap-offsetValueMap);
//      Serial.print("mm   ");
//      Serial.println(offsetValueMap);
//    }
////    offsetValueMap = 0;
//    set_arm_clock( 600000000 );
//    // mag_enc_millis = 0;
//  // }
//}
