#include <Adafruit_SH1106_STM32.h>
#include <SoccerRobot.h>
#include <TDAxis12.h>
#include <Wire.h>
Adafruit_SH1106 display(-1);
SoccerRobot majid;
TwoWire i2c(2, I2C_FAST_MODE);
TDAxis12 gyro(&i2c, 0x10);
float ballAngle = 0;
float Heading = 0;
int SHR, SHB, SHL;
int dif;
int dahan = 0;
int speed = 35000;
int LDR_R, LDR_F, LDR_B, LDR_L;
int LDRL, LDRB, LDRR, LDRF;
int vout = 65535;
int LDRsens = 300;
int d = 0;
bool Shotstate = false;
float Kp = 0.8;
float Ki = 0.01;
float Kd = 1.5;
float integral = 0;
float previousError = 0;
unsigned long previousTime = 0;
int difRotate;

void readSensors() {
  majid.readTSOP();
  ballAngle = majid.getBallAngle();
  majid.updateAngle(Heading);
  //gy
  Heading = gyro.read();
  SHR = analogRead(PA3);
  SHB = analogRead(PA2);
  SHL = analogRead(PA1);
  dif = SHR - SHL;
  dif = dif * 15;

  LDRR = analogRead(PA7) - LDR_R;
  LDRB = analogRead(PA6) - LDR_B;
  LDRL = analogRead(PA5) - LDR_L;
  LDRF = analogRead(PA4) - LDR_F;
  dahan = digitalRead(PB0);
}
void print_All() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("ball:");
  display.println(ballAngle);
  display.print("gy:");
  display.println(Heading);
  display.print("SHR R:");
  display.println(SHR);
  display.print("SHB B:");
  display.println(SHB);
  display.print("SHL L:");
  display.println(SHL);
  display.setCursor(64, 0);
  display.print("LR:");
  display.print(LDRR);
  display.setCursor(64, 10);
  display.print("LB:");
  display.print(LDRB);
  display.setCursor(64, 20);
  display.print("LL:");
  display.print(LDRL);
  display.setCursor(64, 30);
  display.print("LF:");
  display.print(LDRF);
  display.setCursor(64, 40);
  display.print("dahan:");
  display.print(dahan);
  display.setCursor(64, 50);
  display.print("dif:");
  display.print(dif);
  display.display();
}
void moveInside() {
  if (LDRF > LDRsens && LDRL > LDRsens) majid.move(135, vout);
  else if (LDRF > LDRsens && LDRR > LDRsens) majid.move(225, vout);
  else if (LDRB > LDRsens && LDRL > LDRsens) majid.move(45, vout);
  else if (LDRB > LDRsens && LDRR > LDRsens) majid.move(315, vout);
  else if (LDRR > LDRsens) majid.move(270, vout);
  else if (LDRB > LDRsens) majid.move(0, vout);
  else if (LDRL > LDRsens) majid.move(90, vout);
  else if (LDRF > LDRsens) majid.move(180, vout);
  else majid.motor(0, 0, 0, 0);
}
void moveForSec(int dir, int time) {
  majid.move(dir, vout);
  for (int i = 0; i < time; i++) {
    majid.move(dir, vout);
    readSensors();
    print_All();
  }
}
void rotate(bool stat = false) {
  static float filteredDif = 0;
  filteredDif = filteredDif * 0.945 + dif * 0.055;
  if (!stat) {
    majid.setHeading(0);
    return;
  }
  if (abs(filteredDif) < 1500) {
    majid.setHeading(0);
    return;
  }
  float target = filteredDif * 0.0020;
  target = constrain(target, -30, 30);
  majid.setHeading(target);
}
void out() {
  rotate(false);
  int count = 0;
  if (LDRR > LDRsens) {
    moveForSec(270, 5);
    while (ballAngle < 180 && majid.ballDetected() && count < 50) {
      moveInside();
      readSensors();
      print_All();
      count++;
    }
  } else if (LDRB > LDRsens) {
    moveForSec(0, 10);
    while (ballAngle > 90 && ballAngle < 270 && majid.ballDetected() && count < 50) {
      moveInside();
      readSensors();
      print_All();
      count++;
    }
  } else if (LDRL > LDRsens) {
    moveForSec(90, 5);
    while (ballAngle > 180 && majid.ballDetected() && count < 50) {
      moveInside();
      readSensors();
      print_All();
      count++;
    }
  } else if (LDRF > LDRsens) {
    moveForSec(180, 10);
    while ((ballAngle < 90 || ballAngle > 270) && majid.ballDetected() && count < 50) {
      moveInside();
      readSensors();
      print_All();
      count++;
    }
  }
}
void shift() {
  float error = ballAngle;
  if (error > 180)
    error -= 360;
  unsigned long currentTime = micros();
  float dt = (currentTime - previousTime) / 1000000.0;
  if (dt <= 0)
    dt = 0.001;
  integral += error * dt;
  integral = constrain(integral, -100, 100);
  float derivative = (error - previousError);
  float correction = Kp * error
                     + Ki * integral
                     + Kd * derivative;
  correction = constrain(correction, -60, 60);
  if (abs(error) > 10) majid.move(ballAngle + correction, speed);
  else majid.move(0 , speed);
  previousError = error;
  previousTime = currentTime;
}
void Shot(int cycle) {
  if ( dahan == 0 && Shotstate == false) {
    for (int i = 0; i < cycle; i++) {
      readSensors();
      print_All();
      digitalWrite(PB5 , 1);
      digitalWrite(PC13 , 1);
    }
    digitalWrite(PB5 , 0);
    digitalWrite(PC13 , 0);
    delay(100);
    Shotstate = true;
  }
  if (majid.ballDetected() && ballAngle >= 30 && ballAngle <= 330)    Shotstate = false;
}

void setup() {
  majid.begin();
  display.begin(0x3c);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.display();
  majid.muxPins(PA8, PB1, PC14, PC15);
  majid.setHeading(0);
  majid.setRotationPID(400, 25, 400);

  LDR_R = analogRead(PA7);
  LDR_B = analogRead(PA6);
  LDR_L = analogRead(PA5);
  LDR_F = analogRead(PA4);



  pinMode(PC13, OUTPUT);
  pinMode(PB5, OUTPUT);
  digitalWrite(PC13, 0);
  digitalWrite(PB5, 0);
}
void loop() {
  readSensors();
  print_All();
  //rotate();
  Shot(2);
  out();
  if (majid.ballDetected()) {
    rotate(true);
    shift();
  } else {
    rotate(false);
    if (SHB < 1000) majid.motor(-speed - dif, -speed + dif, speed + dif, speed - dif);
    else if (SHB > 1500) majid.motor(speed - dif, speed + dif, -speed + dif, -speed - dif);
    else majid.motor(-dif, dif, dif, -dif);
  }
}
