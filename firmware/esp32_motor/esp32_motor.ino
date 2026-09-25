// ROVERBOT ESP32 motor controller
// F/B/L/R/S = movement/stop
// A### = left PWM, C### = right PWM

#define PWMA 27
#define AIN1 25
#define AIN2 26
#define PWMB 15
#define BIN1 32
#define BIN2 33
#define STBY 12

int leftSpeed = 120;
int rightSpeed = 120;
unsigned long lastMovementCommand = 0;
const unsigned long COMMAND_TIMEOUT_MS = 750;

void stopMotors();
void forward();
void backward();
void turnLeft();
void turnRight();

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  stopMotors();
  lastMovementCommand = millis();
  Serial.println("Rover ready");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    switch (command) {
      case 'F': forward(); lastMovementCommand = millis(); break;
      case 'B': backward(); lastMovementCommand = millis(); break;
      case 'L': turnLeft(); lastMovementCommand = millis(); break;
      case 'R': turnRight(); lastMovementCommand = millis(); break;
      case 'S': stopMotors(); lastMovementCommand = millis(); break;
      case 'A': {
        int newSpeed = Serial.parseInt();
        if (newSpeed >= 0 && newSpeed <= 255) {
          leftSpeed = newSpeed;
          Serial.print("Left PWM = "); Serial.println(leftSpeed);
        }
        break;
      }
      case 'C': {
        int newSpeed = Serial.parseInt();
        if (newSpeed >= 0 && newSpeed <= 255) {
          rightSpeed = newSpeed;
          Serial.print("Right PWM = "); Serial.println(rightSpeed);
        }
        break;
      }
    }
  }
  if (millis() - lastMovementCommand > COMMAND_TIMEOUT_MS) stopMotors();
}

void forward() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, leftSpeed); analogWrite(PWMB, rightSpeed);
}

void backward() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, leftSpeed); analogWrite(PWMB, rightSpeed);
}

void turnLeft() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, leftSpeed / 2); analogWrite(PWMB, rightSpeed);
}

void turnRight() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, leftSpeed); analogWrite(PWMB, rightSpeed / 2);
}

void stopMotors() {
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}
