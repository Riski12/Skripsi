#include <math.h>
#define trigPin1 2    // Trigger
#define echoPin1 3    // Echo
#define trigPin2  4    // Trigger
#define echoPin2  5    // Echo
#define trigPin3  6    // Trigger
#define echoPin3  7    // Echo
#define trigPin4  8    // Trigger
#define echoPin4 9   // Echo
long duration;
int cm1, cm2, cm3, cm4;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //  Sensor ultrasonik 1
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin1, INPUT);
  duration = pulseIn(echoPin1, HIGH);
  // Convert the time into a distance
  cm1 = (duration / 2) / 29.1;   // Divide by 29.1 or multiply by 0.0343
  if (cm1 > 400) {
    cm1 = 400;
    Serial.print ("Kiri = ");
    Serial.println (cm1);
  };

  //  Sensor ultrasonik 2
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  pinMode(echoPin2, INPUT);
  duration = pulseIn(echoPin2, HIGH);
  // Convert the time into a distance
  cm2 = (duration / 2) / 29.1;
  if (cm2 > 400) {
    cm2 = 400;
    Serial.print ("Depan = ");
    Serial.println (cm2);
  };

  //  Sensor ultrasonik 3
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  pinMode(echoPin3, INPUT);
  duration = pulseIn(echoPin3, HIGH);
  // Convert the time into a distance
  cm3 = (duration / 2) / 29.1;
  if (cm3 > 400) {
    cm3 = 400;
    Serial.print ("Kanan = ");
    Serial.println (cm3);
  };

  //  Sensor ultrasonik 4
  digitalWrite(trigPin4, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin4, LOW);
  pinMode(echoPin4, INPUT);
  duration = pulseIn(echoPin4, HIGH);
  // Convert the time into a distance
  cm4 = (duration / 2) / 29.1;
  if (cm4 > 400) {
    cm4 = 400;
    Serial.print ("Belakang = ");
    Serial.println (cm4);
  };

  delay(1000);
}
