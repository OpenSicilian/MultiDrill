//const unsigned int in1 = 6;
//const unsigned int in2 = 7;
const unsigned int dirPin1 = 4;
const unsigned int stepPin1 = 5;
const unsigned int dirPin2 = 7;
const unsigned int stepPin2 = 6;
const unsigned int lim2Pin = 2;
const unsigned int lim1Pin = 3;
const unsigned int IN1 = A0;
const unsigned int IN2 = A1;
const unsigned int IN3 = A2;
const unsigned int IN4 = A3;
const unsigned int button = 8;

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

  //digitalWrite(enA,1);

  pinMode(lim1Pin, INPUT_PULLUP);
  pinMode(lim2Pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(lim1Pin), limitSwitch1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(lim2Pin), limitSwitch2ISR, FALLING);
  Serial.begin(9600);
}



void loop() {

  if (!hasRun) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    // Set direction for both motors
    digitalWrite(dirPin1, CW);
    digitalWrite(dirPin2, CW);

    // Check if limit switch 1 is pressed
    if (digitalRead(lim1Pin) == LOW) {
      // Stop motor 1 if limit switch 1 is pressed
      digitalWrite(stepPin1, LOW);

      hasRun1 = true;
    } else {
      // Move motor 1 if limit switch 1 is not pressed
      digitalWrite(stepPin1, HIGH);
      delayMicroseconds(1000);
      digitalWrite(stepPin1, LOW);
      delayMicroseconds(1000);
    }

    // Check if limit switch 2 is pressed
    if (digitalRead(lim2Pin) == LOW) {
      // Stop motor 2 if limit switch 2 is pressed
      digitalWrite(stepPin2, LOW);
      hasRun2 = true;
    } else {
      // Move motor 2 if limit switch 2 is not pressed
      digitalWrite(stepPin2, HIGH);
      delayMicroseconds(1000);
      digitalWrite(stepPin2, LOW);
      delayMicroseconds(1000);
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
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      MotorControl(1, 0, 200, 6, 1000, CCW);
      //Serial.print("but pressed motor done");
      //Serial.print("has gone in");
      hasRun1 = false;
      hasRun2 = false;
      hasRun = false;
      //Serial.print(hasRun);
    }
  }
}


void MotorControl(bool toggleIn1, bool toggleIn2, unsigned int stepsPerRevolution, unsigned int rev, unsigned int pwm, bool toggleDir) {
  // Set direction for all steppers
  digitalWrite(dirPin1, toggleDir);
  digitalWrite(dirPin2, toggleDir);

  for (int i = 0; i < rev * stepsPerRevolution; i++) {
    // Step HIGH
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);

    delayMicroseconds(pwm);

    // Step LOW
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);

    delayMicroseconds(pwm);

  }
}


void MotorSequence() {

  MotorControl(1, 1, 200, 2, 1000, CW);  //2 was 10

  //MotorControl(1,0,200,2,1000,CW);

  MotorControl(1, 0, 200, 2, 1000, CCW);

  // MotorControl(1,1,200,2,1000,CCW);
}
