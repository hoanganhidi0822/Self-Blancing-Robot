#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#include <TimerOne.h>
#include <Wire.h>
#include <MPU6050.h>


MPU6050 mpu;

#define PWML 3
#define PWMR 11

#define DIR1L 7
#define DIR2L 6
#define DIR1R 9
#define DIR2R 8
#define BTN_PIN 10


/* float Kp = 32.8;   //21.4                   
float Kd = 0.6; //0.0285    verygood p32.1 d0.128 i150 //0.55kd
float Ki = 190; //84 */
float Kp = 60;   //21.4                   
float Kd = 0.9; //0.0285    verygood p32.1 d0.128 i150 //0.55kd
float Ki = 180; //84 

float  targetAngle = 3.8;
float T = 0.007;

int Output_limit  = 255;

volatile float error = 0;
volatile float integral = 0;  
volatile float derivative = 0;
volatile float last_error = 0, errorSum = 0;

int16_t ax, ay, az  ;
int16_t gx, gy, gz;
int16_t gyroX, gyroY, gyroZ;
volatile float accAngle    ;
volatile float gyroRate    ;
volatile float gyroAngle   ;
volatile float currentAngle;
volatile float prevAngle   ;

int16_t accX, accY, accZ;
float gyroAngleX, gyroAngleY, gyroAngleZ;
float accAngleX, accAngleY, accAngleZ;
float compAngleX, compAngleY, compAngleZ;
float alpha = 0.98; // Complementary filter constant
int xungL, xungR, tam;
volatile float goc, goc_fitered, AngleZ;

int Output;

void setup() {

  TCCR2B = TCCR2B & B11111000 | B00000001;  //PWM frequency of 31372.55 Hz
  
  //Interupt Pins
  pinMode(2,INPUT_PULLUP);
  pinMode(10,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  pinMode(5,INPUT_PULLUP);
  attachInterrupt(0,DemxungL,FALLING);
  attachPCINT(digitalPinToPCINT(10), DemxungR, FALLING);

  //Motor control Pins
  pinMode(DIR1L, OUTPUT);
  pinMode(DIR2L, OUTPUT);
  pinMode(DIR1R, OUTPUT);
  pinMode(DIR2R, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);

  Wire.begin();
  mpu.initialize();
  Serial.begin(9600);

  //Timer1 Interupt
  Timer1.initialize(7000);  //don vi us
  Timer1.attachInterrupt(PID);
}

void loop() {
  ax = mpu.getAccelerationX();
  az = mpu.getAccelerationZ(); 
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  

  Serial.println(AngleZ);
  delay(7);
  
  
}

void PID()
{
  
  getAngle();
  // PID CALCULATE
  error = currentAngle - targetAngle;
  errorSum += error;  
  errorSum = constrain(errorSum, -400, 400);


  //Calculate Output from P, I and D values
  Output = Kp*(error) + Ki*(errorSum)*T + Kd*(currentAngle-prevAngle)/T;
  tam = Kd*(currentAngle-prevAngle)/T;
  prevAngle = currentAngle;

  Motor();
  
}
float getAngle(){
  gyroZ /= 156;
  gyroAngleZ += gyroZ * 0.01;
  AngleZ = gyroAngleZ;
  // CALCULATE AND FILTER 
  accAngle = atan2(ax, -az)*RAD_TO_DEG;
  gyroRate = map(gyroY, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*T;  
  
  currentAngle = 0.9934*(prevAngle + gyroAngle) + (1-0.9934)*(accAngle); 
  
}
void Motor(){
  //lIMIT OUTPUT
  Output = constrain(Output, -Output_limit, Output_limit);
  if (Output > 0) {
    // Set the motors to move forward
    digitalWrite(DIR1L, HIGH);
    digitalWrite(DIR2L, LOW);
    digitalWrite(DIR1R, HIGH);
    digitalWrite(DIR2R, LOW);

    analogWrite(PWMR, Output);
    analogWrite(PWML, Output);
  } else if (Output < 0) {
    // Set the motors to move in reverse
    digitalWrite(DIR1L, LOW);
    digitalWrite(DIR2L, HIGH);
    digitalWrite(DIR1R, LOW);
    digitalWrite(DIR2R, HIGH);

    analogWrite(PWMR, abs(Output));
    analogWrite(PWML, abs(Output));
  } else {
    // Stop the motors
    analogWrite(PWMR, 0);
    analogWrite(PWML, 0);
  }
}

void DemxungL()
{
  if(digitalRead(4) == LOW)
  xungL++;
  else
  xungL--;
}

void DemxungR()
{
  if(digitalRead(5) == HIGH)
  xungR ++;
  else
  xungR --;
}




