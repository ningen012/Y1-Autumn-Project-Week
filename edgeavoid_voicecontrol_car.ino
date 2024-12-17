#include <SoftwareSerial.h>
#define TRIG_PIN A1
#define ECHO_PIN A0

int IN1 = 12;
int IN2 = 13;
int IN3 = 3;
int IN4 = 2;
int ENA = 11;
int ENB = 10;

char value;

// Threshold for detecting an edge
const int edgeThreshold = 20; // Distance in cm

void setup() {
  Serial.begin(9600); // For bluetooth
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  
}

long measureDistance() {
  // Send ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(1000);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo time
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Convert to distance in cm
  return duration * 0.034 / 2;

}


void forward()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 80);
  analogWrite(ENB, 80);
  


}

void backward()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 80);
  analogWrite(ENB, 80);


  
}

void right()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, 120);
  analogWrite(ENB, 120);
}

void left()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);//Or right tyre turn anticlockwise
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 120);
  analogWrite(ENB, 120);
}

void stop()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void edgeAvoid(){
  long distance = measureDistance();

  //Serial.println(distance); // Debugging output

  if (distance > edgeThreshold) {
    // Edge detected
    stop();
    delay(500); // Stop for a moment

    backward();
    delay(500); // Move back for 0.5 second

    stop();

  }

  delay(100); // Small delay for stability
}

void voiceControl() {
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);
    if (value == 'f') {
      forward();
    } else if (value == 'b') {
      backward();
      delay(500);
      stop();
    } else if (value == 'l') {
      left();
      delay(500);
      stop();
    } else if (value == 'r') {
      right();
      delay(500);
      stop();

    } else if (value == 's') {
      stop();
    }
  }
}

void loop() {
  edgeAvoid();
  voiceControl();
}