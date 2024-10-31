#include <AFMotor.h>
#include <Servo.h>

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

int trigPin = 24;
int echoPin = 22;

float distance;
float distanceToTarget;
float timeOut = 2 * (150 + 10) / 100 / 340 * 1000000;

float velocity = 0.56;
float left_ang_velocity = 211.5;
float right_ang_velocity = 205.5;
int sdt1 = 500;
int sdt2 = 1000;

bool obstacle = false;
float Dist_To_Obstacle;


Servo myServo;
int Servopin = 26;

void setup() {

  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(Servopin, OUTPUT);
  myServo.attach(Servopin);

  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void loop() {
  myServo.write(90);
  delay(500);
  float left_dist, right_dist;

  Dist_To_Obstacle = Distance();
  while (Dist_To_Obstacle > 8) {
    forward2();
    Dist_To_Obstacle = Distance();
  }

  if (Dist_To_Obstacle <= 8) {

    Stop_1();

    myServo.write(180);
    left_dist = Distance();
    delay(2000);

    myServo.write(0);
    right_dist = Distance();
    delay(2000);

    if (left_dist <= 8 && right_dist > 8) {
      right1(90);
      forward2();
    }
    else if (right_dist <= 8 && left_dist > 8) {
      left1(90);
      forward2();
    }
    else if (right_dist <= 8 && left_dist <= 8) {
      left1(360);
      forward2();
    }
  }
}

/*void forward(float dist, float wv, int dt2) {

  float v;
  float Travel_Time;
  float Motor_Stop_Time = 0;
  bool obstacle = false;
  float Dist_To_Obstacle;
  //float Motor_Start_Time = millis();

  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);

  v = 0.3277 * wv - 2.77;
  Travel_Time = dist / v * 1000;

  float Motor_On_Time = millis() - Motor_Start_Time - Motor_Stop_Time;

  while (Motor_On_Time <= Travel_Time) {

    Motor_On_Time = millis() - Motor_Start_Time - Motor_Stop_Time;
    Dist_To_Obstacle = Distance();
    float initial_time = millis();

    while (Dist_To_Obstacle <= 8) {

      Stop_1();
      Dist_To_Obstacle = Distance();
      obstacle = true;

    }

    float final_time = millis();

    if (obstacle == true) {

      Motor_Stop_Time = Motor_Stop_Time + (final_time - initial_time);
      obstacle = false;

      motor1.run(BACKWARD);
      motor2.run(FORWARD);
      motor3.run(BACKWARD);
      motor4.run(FORWARD);

    }
  }

  Stop_1(); * /
  }*/

void forward1(float dist) {
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);

  float dt = (dist / velocity) * 1000;

  delay(dt);
  Stop_1();
}

void backward1(float dist) {
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);

  float dt = (dist / velocity) * 1000;

  delay(dt);
  Stop_1();
}

void right1(float angle) {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);

  float dt = (angle / right_ang_velocity) * 1000;

  delay(dt);
  Stop_1();
}

void left1(float angle) {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  float dt = (angle / left_ang_velocity) * 1000;

  delay(dt);
  Stop_1();
}

void forward2() {
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}

void backward2() {
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
}

void left2(int dt) {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(dt);
}

void right2(int dt) {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(dt);
}

void Stop(int dt) {

  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  delay(dt);

}

void Stop_1() {

  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);

}


unsigned int ping() {

  unsigned int test_time;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);
  test_time = pulseIn(echoPin, HIGH, timeOut);

  return test_time;

}
float Distance() {

  unsigned int test_time;
  float distance;
  float distanceToTarget;

  test_time = ping();
  distance = (test_time * 1234.*1000.*100) / (3600.*1000000);
  distanceToTarget = distance / 2;

  /*Serial.print("Distance to target is: ");
    Serial.print(distanceToTarget);
    Serial.println(" centimeters. ");
    delay(500);*/

  return distanceToTarget;

}
