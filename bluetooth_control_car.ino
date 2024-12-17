// Motor control pins
int IN1 = 12;
int IN2 = 13;
int IN3 = 3;
int IN4 = 2;
int ENA = 11;
int ENB = 10;

void setup() {
  Serial.begin(9600); // Start Bluetooth communication at 9600 baud
  
  // Motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void loop() {
  
  // Check for incoming Bluetooth commands
  if (Serial.available() > 0) {
    char inputvalue = char(Serial.read());
    Serial.print(inputvalue);
    if (inputvalue == 'F') {//forward
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENA, 80);
      analogWrite(ENB, 80);
    }
    else if (inputvalue == 'B') {//backward
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENA, 80);
      analogWrite(ENB, 80);

    }

    else if (inputvalue == 'R') {//right
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);//Or right tyre turn anticlockwise
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, 120);
  analogWrite(ENB, 120);
  delay(100);
    }

    else if (inputvalue == 'L') {//left
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 120);
  analogWrite(ENB, 120);
  delay(100);
    }
/*
    else if (inputvalue == 'C') {
      digitalWrite(12, LOW);
      digitalWrite(11, HIGH);
      digitalWrite(10, HIGH);
      digitalWrite(9, LOW);
    }

    else if (inputvalue == 'A') {
      digitalWrite(12, HIGH);
      digitalWrite(11, LOW);
      digitalWrite(10, LOW);
      digitalWrite(9, HIGH);
    }

    else if (inputvalue == 'O') {
      digitalWrite(13, HIGH);
    }

    else if (inputvalue == 's') {
      digitalWrite(13, LOW);
    }
*/
    else if (inputvalue == 'S') {//stop
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
    }
  }
}