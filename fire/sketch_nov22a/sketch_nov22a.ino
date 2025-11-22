#include <Arduino.h>
#include <Servo.h>

// Motor Driver Pins
#define ENA1 4
#define IN11 5
#define IN21 6
#define ENA2 7
#define IN12 8
#define IN22 9

// Pump
#define pump 20

// Ultrasonic Sensor
#define trigPin1 10
#define echoPin1 11

#define trigPin2 26
#define echoPin2  25

// Servo
#define servoPin 22

// Flame Sensors
#define flame1D0 12
#define flame2D0 21   // Fixed: cannot use same pin as pump

// Indicators
#define ledPin 13
#define buzzer 18

Servo myservo;
int lastServoPos = 90; // Middle position

// -------------------------- SETUP --------------------------
void setup()
{
  // Motors
  pinMode(ENA1, OUTPUT);
  pinMode(IN11, OUTPUT);
  pinMode(IN21, OUTPUT);
  pinMode(ENA2, OUTPUT);
  pinMode(IN12, OUTPUT);
  pinMode(IN22, OUTPUT);

  // Ultrasonic
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);


  // Flames
  pinMode(flame1D0, INPUT);
  pinMode(flame2D0, INPUT);

  // Indicators + Pump
  pinMode(ledPin, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(pump, OUTPUT);

  myservo.attach(servoPin);
  myservo.write(lastServoPos);

  digitalWrite(pump, LOW);
}

// -------------------------- MAIN LOOP --------------------------
void loop()
{
  int flame1State = digitalRead(flame1D0);
  int flame2State = digitalRead(flame2D0);

  // No flame → move normally (you can add obstacle avoidance)
  if (flame1State == LOW && flame2State == LOW) {
    float distanceFront = calculateDistanceInTheFront();
    float distanceRight = calculateDistanceInTheRight();

    if(distanceFront  < 30){
      stopMovement();
      delay(200);
      if(distanceRight < 30){
        turnLeft(200);
        delay(500);
      } else {
        turnRight(200);
        delay(500); 
      }
    } else {
      moveForward(200);
    }
    return;
  }

  // Flame Detected → Stop & Alert
  stopMovement();
  digitalWrite(ledPin, HIGH);
  digitalWrite(buzzer, HIGH);

  // ------------------ FLAME RIGHT ONLY ------------------
  if (flame1State == HIGH && flame2State == LOW)
  {
    sweepTo(0);
    sprayUntil(flame1D0);
  }

  // ------------------ FLAME LEFT ONLY ------------------
  else if (flame2State == HIGH && flame1State == LOW)
  {
    sweepTo(180);
    sprayUntil(flame2D0);
  }

  // ------------------ FLAME ON BOTH SIDES ------------------
  else if (flame1State == HIGH && flame2State == HIGH)
  {
    // Left first
    sweepTo(180);
    sprayUntil(flame2D0);

    delay(300);

    // Then right
    sweepTo(0);
    sprayUntil(flame1D0);
  }

  digitalWrite(ledPin, LOW);
  digitalWrite(buzzer, LOW);
}


void sweepTo(int angle)
{
  if (lastServoPos != angle) {
    myservo.write(angle);
    delay(400);
    lastServoPos = angle;
  }
}

void sprayUntil(int flamePin)
{
  while (digitalRead(flamePin) == HIGH)
  {
    digitalWrite(pump, HIGH);
    delay(300);
    digitalWrite(pump, LOW);
    delay(200);
  }
}

float calculateDistanceInTheFront()
{
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  long duration = pulseIn(echoPin2, HIGH);
  float distance = duration * 0.034 / 2;
  return distance;
}

float calculateDistanceInTheRight()
{
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  long duration = pulseIn(echoPin1, HIGH);
  float distance = duration * 0.034 / 2;
  return distance;
}

void moveForward(int speed)
{
  analogWrite(ENA1, speed);
  digitalWrite(IN11, HIGH);
  digitalWrite(IN21, LOW);

  analogWrite(ENA2, speed);
  digitalWrite(IN12, HIGH);
  digitalWrite(IN22, LOW);
}

void stopMovement()
{
  analogWrite(ENA1, 0);
  analogWrite(ENA2, 0);

  digitalWrite(IN11, LOW);
  digitalWrite(IN21, LOW);
  digitalWrite(IN12, LOW);
  digitalWrite(IN22, LOW);
}

void turnRight(int speed)
{
  analogWrite(ENA1, speed);
  digitalWrite(IN11, HIGH);
  digitalWrite(IN21, LOW);

  analogWrite(ENA2, speed);
  digitalWrite(IN12, LOW);
  digitalWrite(IN22, HIGH);
}

void turnLeft(int speed)
{
  analogWrite(ENA1, speed);
  digitalWrite(IN11, LOW);
  digitalWrite(IN21, HIGH);

  analogWrite(ENA2, speed);
  digitalWrite(IN12, HIGH);
  digitalWrite(IN22, LOW);
}
