//const unsigned int in1 = 6;
//const unsigned int in2 = 7;
const unsigned int dirPin1 = 4;   //4//pin1
const unsigned int stepPin1 = 5;  //5pin1
const unsigned int dirPin2 = 7;   //7
const unsigned int stepPin2 = 6;  //6
const unsigned int lim2Pin = 2;
const unsigned int lim1Pin = 3;
const unsigned int IN1 = A0;
const unsigned int IN2 = A1;
const unsigned int IN3 = A2;
const unsigned int IN4 = A3;
const unsigned int button = 8;
const unsigned int ENB = 9;
const unsigned int ENA = A4;


bool CW = 1;
bool CCW = 0;

// State Variables
volatile bool limitSwitch1Triggered = false;
volatile bool limitSwitch2Triggered = false;
volatile bool initiation = false;
volatile bool test = false;
volatile bool mot1Z = false;
volatile bool mot2Z = false;
bool hasRun = false;
bool hasRun1 = false;
bool hasRun2 = false;
bool drillReturn = false;
const unsigned int microDelay = 700;

unsigned long lastMillis = 0;
const unsigned long debounceDelay = 50;  // 50 ms debounce delay

// Interrupt Service Routines
void limitSwitch1ISR() {
  limitSwitch1Triggered = true;
}

void limitSwitch2ISR() {
  limitSwitch2Triggered = true;
}

void setup() {
  // put your setup code here, to run once:

  //pinMode(enA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(13, OUTPUT);  //LED
  pinMode(button, INPUT_PULLUP);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);


  //digitalWrite(enA,1);

  pinMode(lim1Pin, INPUT_PULLUP);
  pinMode(lim2Pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(lim1Pin), limitSwitch1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(lim2Pin), limitSwitch2ISR, FALLING);
  // Serial.begin(9600);
  analogWrite(ENA, 250);
  analogWrite(ENB, 250);
}



void loop() {

  if (!hasRun) {
    if (!drillReturn) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    } else {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }

    // Set direction for both motors
    digitalWrite(dirPin1, CW);
    digitalWrite(dirPin2, CW);

    // Check if limit switch 1 is pressed
    if (digitalRead(lim1Pin) == LOW) {
      // Stop motor 1 if limit switch 1 is pressed
      digitalWrite(stepPin1, LOW);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);

      hasRun1 = true;
    } else {
      // Move motor 1 if limit switch 1 is not pressed
      digitalWrite(stepPin1, HIGH);
      delayMicroseconds(microDelay);
      digitalWrite(stepPin1, LOW);
      delayMicroseconds(microDelay);
    }

    // Check if limit switch 2 is pressed
    if (digitalRead(lim2Pin) == LOW) {
      // Stop motor 2 if limit switch 2 is pressed
      digitalWrite(stepPin2, LOW);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      hasRun2 = true;
    } else {
      // Move motor 2 if limit switch 2 is not pressed
      digitalWrite(stepPin2, HIGH);
      delayMicroseconds(microDelay);
      digitalWrite(stepPin2, LOW);
      delayMicroseconds(microDelay);
    }
    if (hasRun1 == true && hasRun2 == true) {
      hasRun = true;
    }
  }
  if (hasRun) {
    //Serial.print("pasrun true works");
    //Button search here
    bool drill = digitalRead(button);
    delay(50);
    if (drill == LOW) {
      //Serial.print("about to go in");
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      delay(125);

      MotorControlDual(900, 500, CCW, 135, 1000, CCW);  //good speed final crnrs good no sides
      MotorControlDual(330, 7500, CCW, 270, 10000, CCW);  //good push finalcrnrs good no sides

      //MotorControlDual(900, 500, CCW, 900, 500, CCW);  // final - single 
     // MotorControlDual(330, 5000, CCW, 330, 5000, CCW);  //final - single
      

      hasRun1 = false;
      hasRun2 = false;
      hasRun = false;
      drillReturn = true;
    }
  }
}


void MotorControlDual(
  unsigned int steps1, unsigned int pwm1, bool dir1,
  unsigned int steps2, unsigned int pwm2, bool dir2) {
  digitalWrite(dirPin1, dir1);
  digitalWrite(dirPin2, dir2);

  unsigned long currentTime;
  unsigned long nextStepTime1 = micros();
  unsigned long nextStepTime2 = micros();

  unsigned int stepsDone1 = 0;
  unsigned int stepsDone2 = 0;

  bool stepState1 = false;
  bool stepState2 = false;

  while (stepsDone1 < steps1 || stepsDone2 < steps2) {
    currentTime = micros();

    // Motor 1 step control
    if (stepsDone1 < steps1 && currentTime >= nextStepTime1) {
      digitalWrite(stepPin1, stepState1);
      stepState1 = !stepState1;
      if (!stepState1) stepsDone1++;  // count step on falling edge
      nextStepTime1 = currentTime + pwm1;
    }

    // Motor 2 step control
    if (stepsDone2 < steps2 && currentTime >= nextStepTime2) {
      digitalWrite(stepPin2, stepState2);
      stepState2 = !stepState2;
      if (!stepState2) stepsDone2++;  // count step on falling edge
      nextStepTime2 = currentTime + pwm2;
    }
  }
}
