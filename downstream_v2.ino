#include <AccelStepper.h>
#include <MultiStepper.h>
#include <elapsedMillis.h>


//pins
int ledPin = 12;
int hallPin = 3;
int barrierPin = 2;
int dirPin = A5;
int stepPin = A4;
int sleepPin = A3;

//states
unsigned int state;

unsigned const int STATE_START = 0;

unsigned const int STATE_HOMING = 2;
unsigned const int STATE_HOMING_1 = 3;
unsigned const int STATE_HOMING_TO_RUNNING = 7;

unsigned const int STATE_RUNNING = 1;
unsigned const int STATE_RUNNING_1 = 101;


unsigned const int STATE_EMPTY = 8;
unsigned const int STATE_EMPTY_1 = 801;



unsigned const int STATE_SLEEP = 9;
unsigned const int STATE_SLEEP_1 = 901;


unsigned const int STATE_ERROR = 100;


unsigned long position = 0;
unsigned long ballsDone = 0;
int ballsToGo = 0;
unsigned int speed = 0;
const int MAX_SPEED = 1400;

//stepper
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

//stepper sleep
boolean isStepperEnabled = true;
elapsedMillis sleepTimer;
unsigned long SLEEP_TIME = 1000 * 3;

const int FALL_OFFSET = 8 * 80;

//hall sensor
boolean isHallSensorEnabled;

//timing
elapsedMillis runningTimeout = 0;

//debug

boolean debug = true;

void dPrint(String s) {
  if(!debug) return;
  Serial.print(s);
}

void dPrintln(String s) {
  if(!debug) return;
  Serial.println(s);
}


void setup() {
  Serial.begin(9600);
  Serial.setTimeout(5);

  //setup pins
  pinMode(ledPin, OUTPUT);

  initStepper();
  initHallSensor();
  initLightBarrier();

  state = STATE_START;
}

void initLightBarrier() {
  pinMode(barrierPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(barrierPin), updateLightBarrier, RISING);
}

void initHallSensor() {
  pinMode(hallPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(hallPin), updateHallSensor, CHANGE);

}

void updateHallSensor() {
  isHallSensorEnabled = !digitalRead(hallPin);
}



void updateLightBarrier() {
  if (state == STATE_RUNNING_1) {
    runningTimeout = 0;
    ballsToGo--;  
    ballsDone++;
  }
}

void initStepper() {
  stepper.setEnablePin(sleepPin);
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.enableOutputs();
  stepper.setAcceleration(2000);
}


void loop() {
  
  if(state == STATE_START) {
    stepper.enableOutputs();
    digitalWrite(ledPin, LOW);
    isStepperEnabled = true;
    ballsToGo = 0;
    sleepTimer = 0;
    delay(200);
    dPrintln("homing...");
    state = STATE_HOMING;  
  }

  if (state == STATE_HOMING) {

    if (!isHallSensorEnabled) {
      stepper.setSpeed(1000);
      stepper.runSpeed();
    } else {
      state = STATE_HOMING_1;
      dPrintln("calibration point found.");

    }
  }

  if (state == STATE_HOMING_1) {
    if (isHallSensorEnabled) {

      stepper.moveTo(stepper.currentPosition() - 1);
    } else {
      stepper.moveTo(stepper.currentPosition() + 1);
      stepper.setCurrentPosition(0);

      state = STATE_HOMING_TO_RUNNING;
      dPrintln("calibration set zero.");
    }
  }

  if (state == STATE_HOMING_TO_RUNNING) {
    dPrintln("move to offset.");

    //blocking
    stepper.runToNewPosition(FALL_OFFSET);

    state = STATE_RUNNING;
  }

  if (state == STATE_RUNNING) {
    runningTimeout = 0;
    stepper.enableOutputs();
    state = STATE_RUNNING_1;
  }


  if (state == STATE_RUNNING_1) {

    if (Serial.available() > 0) {
      int in = Serial.parseInt();

      if (in > 0) {
        sleepTimer = 0;
        ballsToGo += in;
      }
    }

    if (sleepTimer > SLEEP_TIME) {  
        state = STATE_SLEEP;
    }

    if(ballsToGo > 0) {
      sleepTimer = 0;
      //calculate speed
      // 200 rv * 16 stps = 3200 stps pr rv.
      // steps per ball =  s/b
      // 3200 / 40 = 80 s/b
      unsigned int newSpeed = ballsToGo * 80;
      
      if(newSpeed > MAX_SPEED) {
        newSpeed = MAX_SPEED;
      }
      // if lanes are empty increase speed
      if (runningTimeout > 3000) {  
        newSpeed = MAX_SPEED;
      }
      
      if(newSpeed != speed) {
        speed = newSpeed;
        dPrint("adjusted speed: ");
        dPrint(String(speed, DEC));
        dPrint(" steps/sec, ");
        dPrint(String(ballsToGo, DEC));
        dPrint("/");
        dPrintln(String(ballsDone, DEC));
      }
  
      if(stepper.currentPosition() % 80 < 10) {
        stepper.setSpeed(80); 
      } else {    
        stepper.setSpeed(speed);
        
      }
    } else {
      runningTimeout = 0;
      stepper.setSpeed(0);
      if(ballsToGo < 0) {
        digitalWrite(ledPin, HIGH);
      } else {
        digitalWrite(ledPin, LOW);
      }
      
    }
  }

  if(state == STATE_SLEEP) {
    dPrintln("go to sleep.");
    stepper.disableOutputs();
    isStepperEnabled = false;
    delay(200);
    state = STATE_SLEEP_1;
  }

  if(state == STATE_SLEEP_1) {
    if (Serial.available() > 0) {
      state = STATE_START;
    }
  }

  if (state == STATE_EMPTY) {
    dPrintln("empty.");

    digitalWrite(ledPin, HIGH);
    stepper.disableOutputs();
    isStepperEnabled = false;
    delay(200);
    state = STATE_EMPTY_1;
  }

  if(state == STATE_EMPTY_1) {
    if (Serial.available() > 0) {
      int in = Serial.parseInt();
      
      if(in == -9) {
        //reset machine
        dPrintln("reset");
        state = STATE_START;
      }
    }
  }

  if (isStepperEnabled && state != STATE_EMPTY) {
    stepper.run();
  }
}


