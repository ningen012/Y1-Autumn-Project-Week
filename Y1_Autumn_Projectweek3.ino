#include <LiquidCrystal.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


// Motor control pins
int IN1 = 12;
int IN2 = 13;
int IN3 = 3;
int IN4 = 2;
int ENA = 11;
int ENB = 10;
const int LEFT_SENSOR= A1;
const int RIGHT_SENSOR= A2;

// LCD pins
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Encoder pin
#define rotaryRight A0

// Wheel and encoder parameters
const float pi = 3.14159;
const float R = 3.78;              // Radius of the wheel in cm 3.25
const int PPR = 20;                // Pulses per revolution
const float distancePerPulse = (2 * pi * R) / PPR;

int counter = 0;                   // Count pulses from encoder
float totalDistance = 0.0;         // Total accumulated distance
int lastRotaryRight = LOW;         // Previous state of encoder signal


Adafruit_MPU6050 mpu;
const float pitchThreshold = 2.0; // Threshold for detecting a ramp
float maxPitch = 0.0;

// Variables
unsigned long startTime = 0;       // To track elapsed time
unsigned long stopTime = 0;        // Timer for 3-second stop
bool stopFor3Sec = false;          // Flag for 3-second stop
bool pathCompleted = false;        // Flag for completing 200 cm path
bool motorRunning = true;          // To control motor state
bool isRunning = true;             // Motor running state





void setup() {
  // Motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(LEFT_SENSOR,INPUT);// Define A1 pin for the IR sensor mine
  pinMode(RIGHT_SENSOR,INPUT);//A2 for IR sensor 2

  // Encoder setup
  pinMode(rotaryRight, INPUT);

  // LCD setup
  lcd.begin(16, 2);


  // Initialize variables
  totalDistance = 0.0;
  counter = 0;
  startTime = millis(); // Record the starting time
  motorRunning = true;
  

Serial.begin(115200);

  while (!Serial)
    delay(10);

 // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
   while (1) {
      delay(10);
    }
  }
  else {Serial.println("MPU6050 Initialized!");}

  // Configure MPU6050 settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);



}




void forward()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 80);
  analogWrite(ENB, 80);


}

void backward()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 80);
  analogWrite(ENB, 80);


  
}

void right()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);//right tyre turn anticlockwise
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, 120);
  analogWrite(ENB, 120);
  delay(100);
}

void left()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 120);
  analogWrite(ENB, 120);
  delay(100);
}

void stop()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void lineFollowing(){
  int RIGHT_SENSORdigital = digitalRead(RIGHT_SENSOR);
  int LEFT_SENSORdigital = digitalRead(LEFT_SENSOR);
  int LEFT_SENSORanalog = analogRead(LEFT_SENSOR);
  int RIGHT_SENSORanalog = analogRead(RIGHT_SENSOR);
  Serial.print("Right Digital: "); Serial.print(RIGHT_SENSORdigital);
  Serial.print(", Left Digital: "); Serial.println(LEFT_SENSORdigital);
  Serial.print("Right Analog: "); Serial.print(RIGHT_SENSORanalog);
  Serial.print(", Left Analog: "); Serial.println(LEFT_SENSORanalog);

  if (abs(LEFT_SENSORanalog - RIGHT_SENSORanalog) < 50) { // Ignore minor deviations
    forward();
  }

  if (RIGHT_SENSORdigital == HIGH && LEFT_SENSORdigital == HIGH) {
    forward();
  } else if (RIGHT_SENSORdigital == LOW && LEFT_SENSORdigital == HIGH) {
    right();
  } else if (RIGHT_SENSORdigital == HIGH && LEFT_SENSORdigital == LOW) {
    left();
  } else if (RIGHT_SENSORdigital == LOW && LEFT_SENSORdigital == LOW) {
    stop();
    isRunning = false;
  }
}

void displayInfo(float distance) {

   // Display distance on the first row
  lcd.setCursor(0, 1);
  lcd.print("Distance: ");
  lcd.print(distance, 2);
  lcd.print("cm   "); // Clear remaining characters

}

void displayFinalStats(float distance) {
  // Display final distance on the first row
  lcd.setCursor(0, 1);
  lcd.print("Final Dist: ");
  lcd.print(distance, 2);
  lcd.print("cm   "); // Clear remaining characters

}


void loop() {
  static unsigned long rampStopTime = 0;       // To track the time at the top of the ramp
  static bool topOfRamp = false;              // Flag to indicate the robot is at the top of the ramp
  static bool rotated = false;                // Flag to indicate if the robot has completed rotation
  static bool descendingRamp = false;         // Flag to indicate the robot is descending the ramp
  static bool sequenceComplete = false;       // Flag to ensure the sequence runs only once
  static bool lineFollowMode = false;

  static float angleSum = 0.0;                // Sum of ramp angles
  static int angleCount = 0;                  // Count of angle readings
  static float averageAngle = 0.0;            // Initialize to 0.0

  unsigned long currentTime = millis();
  int elapsedTime = (currentTime - startTime) / 1000; // Elapsed time in seconds

  lcd.setCursor(0, 0);
  lcd.print("Time: ");
  lcd.print(elapsedTime);
  lcd.print("s      ");

  // If the sequence is complete, switch to line-following mode
  if (sequenceComplete && !lineFollowMode) {
    lineFollowMode = true;
  }

  if (lineFollowMode) {
    // Ensure distance and stopping logic are handled in line-following mode
    int currentRotaryRight = digitalRead(rotaryRight);

    // Count pulses and calculate distance
    if (lastRotaryRight == LOW && currentRotaryRight == HIGH) {
      counter++;
      totalDistance += distancePerPulse;
      delay(10);
    }
    lastRotaryRight = currentRotaryRight;

    // Stop for 3 seconds after 200 cm
    if (!pathCompleted && totalDistance >= 200.0) {
      stop();
      pathCompleted = true;
      stopFor3Sec = true;
      stopTime = millis();
      displayFinalStats(totalDistance);
      return;
    }

    // Resume after the 3-second stop
    if (stopFor3Sec) {
      if (currentTime - stopTime >= 3000) {
        stopFor3Sec = false; // Reset stop flag
        isRunning = true;
      } else {
        stop();
        return;
      }
    }

    // Display distance
    displayInfo(totalDistance);

    // Perform line-following logic
    lineFollowing();

    // Stop when both sensors detect the end
    if (digitalRead(LEFT_SENSOR) == LOW && digitalRead(RIGHT_SENSOR) == LOW) {
      stop();
      isRunning = false;
      displayFinalStats(totalDistance);
      while (true); // Stop loop permanently
    }
    return; // Ensure no further code runs in this iteration
  }

  // Get sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate pitch angle
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  // Ramp detection and handling
  if (!topOfRamp) {
    if (pitch > pitchThreshold) {//pitchThreshold=2.0
      forward();
      analogWrite(ENA, 200); // Increase speed on ramp
      analogWrite(ENB, 200);
      delay(300);

      // Accumulate ramp angles if above the threshold
      if (pitch > 20.0) {
        angleSum += pitch;
        angleCount++;

      }
    } else {
      forward();
      analogWrite(ENA, 80); // Restore normal speed
      analogWrite(ENB, 80);
    }

    if (pitch < pitchThreshold && angleCount > 0) {
      topOfRamp = true;
      rampStopTime = millis();
      stop();

      // Calculate average angle
      averageAngle = angleSum / angleCount;
    }
  } else if (millis() - rampStopTime < 4000) {
    stop(); // Stop for 4 seconds at the top of the ramp
    lcd.setCursor(0, 0);
    lcd.print("Time:");
    lcd.print(elapsedTime);
    lcd.setCursor(0, 1);
    lcd.print("Angle: ");
    lcd.print(averageAngle, 2);
    lcd.print("   ");
  } else if (!rotated) {
    // Perform 360° rotation
    float accumulatedYaw = 0.0;
    unsigned long prevTime = millis();

    while (fabs(accumulatedYaw) < 360.0) {
      right();
      analogWrite(ENA, 100);
      analogWrite(ENB, 100);

      mpu.getEvent(&a, &g, &temp);
      unsigned long currentTime = millis();
      float deltaTime = (currentTime - prevTime) / 1000.0;
      prevTime = currentTime;
      accumulatedYaw += g.gyro.z * 180.0 / PI * deltaTime;
    }
    
    rotated = true;
  
  } else if (!descendingRamp) {
    forward();
    analogWrite(ENA, 60); // Normal speed
    analogWrite(ENB, 60);

    if (pitch < -10) {
      descendingRamp = true;
    }
  } else {
    forward();
    analogWrite(ENA, 50);
    analogWrite(ENB, 50);

    if (pitch > 0) { // Back on level ground
      sequenceComplete = true;
    }
  }
}

