void init() {
  // serial setup
  Serial.begin(115200);
  inputString.reserve(200);  // reserve 200 bytes for the inputString:

  // stepper driver setups
  analogWriteFrequency(STEPPER_I1, PWM_FREQ);
  for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
    steppers[i]->setMaxSpeed(STEPPER_SPEED);
    steppers[i]->setAcceleration(STEPPER_ACC);
  }
  pinMode(STEPPER_I1, OUTPUT);
  analogWrite(STEPPER_I1, 250);
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
  reset_steppers();

  // rotary encoder setup
  pinMode(ROTARY_ENC_COM, OUTPUT);
  digitalWrite(ROTARY_ENC_COM, LOW);

  // buttons setup
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



void fetch_mag_val() {
  // if (mag_enc_millis >= mag_enc_delay) {
    set_arm_clock( 24000000 );
    //Step 1: Get the value
    encoderVal = as5311.value();

    //Step 2: Change direction
    encoderVal = map(encoderVal, 0, 4095, 4095, 0);

    //Step 3: Get the zero error and use dynamic zeroing
    encoderVal = dynamicZeroError();

    //Step 4: Increment or decrement counter variable
    checkCounter();

    //Step 5: Assign a proper value
    valueMap = (((counter * 2000) + encoderVal * 0.488) / 1000);

    //Show some data
    //      showData();
    as5311.error();
    if (!as5311.err_value.DECn) { // if there's not a decn error
      Serial.print(valueMap-offsetValueMap);
      Serial.print("mm   ");
      Serial.println(offsetValueMap);
    }
//    offsetValueMap = 0;
    set_arm_clock( 600000000 );
    // mag_enc_millis = 0;
  // }
}



void button_handler() {
  left_but.update();
  right_but.update();
  up_but.update();
  down_but.update();
  rotary_but.update();
  if ( left_but.fell() ) {
    reset_steppers();
    
    stepper_driver_power(1);
    for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
      steppers[i]->setMaxSpeed(200);
      steppers[i]->moveTo(6000);
    }
    bool flag = false;
    while (!flag) {
      for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
        fetch_mag_val();
//        Serial.println(steppers[i]->currentPosition());
        if ((valueMap  - offsetValueMap>= 6) || (steppers[i]->currentPosition() >=4000)) { // second condition not meeting
          flag = true;
          break;
        }
        steppers[i]->run();
      }
    }
    for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
      steppers[i]->stop();
      steppers[i]->setMaxSpeed(STEPPER_SPEED);
    }
    


  }
  if ( right_but.fell() ) {

    if (shape_num < 2)
      shape_num++;
    else
      shape_num = 0;
    set_shape();
    Serial.print("shape_num "); Serial.println(shape_num);

  }
  if ( up_but.fell() ) {
    if (stepper_index < NUM_OF_STEPPERS - 1)
      stepper_index++;
    Serial.print("Stepper index: "); Serial.println(stepper_index);
  }
  if ( down_but.fell() ) {
    if (stepper_index > 0)
      stepper_index--;
    Serial.print("Stepper index: "); Serial.println(stepper_index);
  }
  if ( rotary_but.fell() ) {
    stepper_driver_power(!isStepperEnabled);
    Serial.print("Steppers enable: "); Serial.println(isStepperEnabled);
  }
}




void reset_steppers() {
  stepper_driver_power(1);
  for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
    steppers[i]->moveTo(-4000);
  }
  unsigned long time_now = millis();
  while (millis() - time_now <= 2000) {
    for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
      steppers[i]->run();
    }
  }
  for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
    steppers[i]->setCurrentPosition(0);
  }
  delay(200);
  for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
    steppers[i]->moveTo(STEPPER_OFFSET);
  }
  time_now = millis();
  while (millis() - time_now <= 200) {
    for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
      steppers[i]->run();
    }
  }
  for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
    steppers[i]->setCurrentPosition(0);
  }
  stepper_driver_power(0);
  fetch_mag_val();
  offsetValueMap = valueMap;
  
  
  delay(5);
}








void stepper_driver_power(bool in) {
  isStepperEnabled = in;
  digitalWrite(STEPPER_SLEEP, isStepperEnabled);
  digitalWrite(DEBUG_LED, isStepperEnabled);
}


void set_shape() {
  reset_steppers();
  int count = 0;
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      switch (shape_num) {
        case 0:
          steppers[count]->moveTo(shape_one[i][j]);
          break;
        case 1:
          steppers[count]->moveTo(shape_two[i][j]);
          break;
        case 2:
          steppers[count]->moveTo(shape_three[i][j]);
      }
      count++;
    }
  }
  stepper_driver_power(1);

  unsigned long time_now = millis();
  while (millis() - time_now <= 2000) {
    for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
      steppers[i]->run();
    }
  }
  for (uint8_t i = 0; i < NUM_OF_STEPPERS; i++) {
    steppers[i]->setCurrentPosition(0);
  }
  stepper_driver_power(0);
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


void checkCounter() {

  uint32_t triPole = as5311.triPole(encoderVal);

  if (lastPole == 2 && triPole == 0) {
    counter++;
  }

  if (lastPole == 0 && triPole == 2) {
    counter--;
  }

  lastEncoderVal = encoderVal;
  lastPole = triPole;
}



int dynamicZeroError() {
  if (Serial.available() > 0) {
    // read the incoming byte:
    int readVal = Serial.read();

    if (readVal == 48)
    {
      zeroVal = encoderVal;
      finalPoint = 4095 - encoderVal;
    }
  }

  return as5311.dynamic_zero(encoderVal, zeroVal, finalPoint);
}



void showData() {

  Serial.print(encoderVal);
  Serial.print(" ");

  Serial.print(valueMap / 1000);
  Serial.print("mm ");

  if (as5311.error()) {

    if (as5311.err_value.OCF) {
      Serial.print(" OCF");
    } else {
      Serial.print("    ");
    }
    if (as5311.err_value.COF) {
      Serial.print(" COF");
    } else {
      Serial.print("    ");
    }
    if (as5311.err_value.DECn) {
      Serial.print(" DEC");
    } else {
      Serial.print("    ");
    }
    if (as5311.err_value.INCn) {
      Serial.print(" INC");
    } else {
      Serial.print("    ");
    }
    if (as5311.err_value.LIN) {
      Serial.print(" LIN");
    } else {
      Serial.print("    ");
    }

    Serial.println();
  }
}
