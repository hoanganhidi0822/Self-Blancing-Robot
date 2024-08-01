#include <SimpleKalmanFilter.h>
SimpleKalmanFilter bo_loc(2, 2, 0.001);
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


//////---PID variable---//////
float Kp = 60;   //21.4                   
float Kd = 0.9; //0.0285    verygood p32.1 d0.128 i150 //0.55kd
float Ki = 290; //84 

float Kpy = 4.2; 
float Kdy = 0.008; 
float Kiy = 2.0; 

float Kpp = 0.09; 
float Kdp = 9.8; 
float Kip = 0.1; 

float T = 0.007; // SAMPLE TIME donvi s

float  targetAngle = 4.4;
float  targetYaw = 0;

int Output_limit  = 255;

volatile float error = 0;
volatile float integral = 0;  
volatile float derivative = 0;
volatile float last_error = 0, errorSum = 0;


//////---MPU variable---//////
int16_t ax, ay, az  ;
int16_t gx, gy, gz;
int16_t gyroX, gyroY, gyroZ;
volatile float accAngle    ;
volatile float gyroRate    ;
volatile float gyroAngle   ;
volatile float currentAngle;
volatile float prevAngle   ;
volatile float goc, goc_fitered, z;

int16_t accX, accY, accZ;
float gyroAngleX, gyroAngleY, gyroAngleZ;
float accAngleX, accAngleY, accAngleZ;
float compAngleX, compAngleY, compAngleZ;
float alpha = 0.9934; // Complementary filter constant
float AngleZ ;

float xungL, xungR;
int setpoint, setpointYaw;
float position = 0;
int targetPosition = 0;
int setpointP;

int Output;
int OutputY;
int OutputP;
float OutputPx;

float Pitch_trim = 0;
float Yaw_trim = 0;
float Yaw_trimx = 0;
int speedRight;
int speedLeft;
float filteredValue = 0, ALPHA = 0.02;
float R = 3.5;

#define RCPin 2
volatile long time_Star = 0, time_Star1 = 0;
volatile long xung = 0, xung1 = 0;
int rongxung = 0, Pitch = 0, Yaw = 0;

 float errorY;
 float currentAngleY;
float errorSumY;  
float prevAngleY;

 float errorP;
 float currentAngleP;
float errorSumP;  
float prevAngleP;


void setup() {

  TCCR2B = TCCR2B & B11111000 | B00000001;  //PWM frequency of 31372.55 Hz
  
  //Interupt Pins
  pinMode(2,INPUT_PULLUP);
  pinMode(10,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  pinMode(5,INPUT_PULLUP);
  pinMode(12,INPUT_PULLUP);
  pinMode(13,INPUT_PULLUP);
  attachInterrupt(0,DemxungL, FALLING);
  attachPCINT(digitalPinToPCINT(10), DemxungR, FALLING);
 
  ///Read Reciever value
  attachPCINT(digitalPinToPCINT(12), Pulse_Pitch, CHANGE);
  attachPCINT(digitalPinToPCINT(13), Pulse_Yaw, CHANGE);

  //Motor control Pins
  pinMode(DIR1L, OUTPUT);
  pinMode(DIR2L, OUTPUT);
  pinMode(DIR1R, OUTPUT);
  pinMode(DIR2R, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);

  Wire.begin();
  mpu.initialize();
  Serial.begin(115200);

  //Timer1 Interupt
  Timer1.initialize(T*1000000);  //don vi us
  Timer1.attachInterrupt(PID);
}

void loop() {
  ax = mpu.getAccelerationX();
  az = mpu.getAccelerationZ(); 
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  
  //////////////////Position////////////////
  
  
  if ((Pitch_trim > 2 || Pitch_trim < -2)){
    targetPosition = Pitch_trim;
   
    xungR = 0;
    xungL = 0;
    Yaw_trimx = 0;
  }else {
    position = ((xungL + xungR)*0.5/330)*2*PI*32.5;
    //position = constrain(position, -500, 500);
    targetPosition = 0;

  }
  
  
  if (Yaw_trim > 2 || Yaw_trim < - 2){
     AngleZ = 0;
     xungR = 0;
     xungL = 0;
  }
  else{

    gyroZ /= 156;
    gyroAngleZ += gyroZ * 0.01;
    AngleZ = gyroAngleZ;
    AngleZ = constrain(AngleZ, -18, 18);

   
  }

  ////////////////Print///////////////////
 
  
  Serial.println(position);
 
  
  ////////////////////////////////////////
  Motor();
}

void PID()
{
  getAngle();
  
  ////////////--- CAL PID---/////////////
  ///ERROR Position###############################

  setpointP = targetPosition - position; 
  errorP = currentAngleP - setpointP;
  errorSumP += errorP;  
  errorSumP = constrain(errorSumP, -200, 200);
  OutputP = Kpp*(errorP) + Kip*(errorSumP)*T + Kdp*(currentAngleP-prevAngleP)/T;
  prevAngleP = currentAngleP;
  OutputPx = constrain(OutputP, -3.6, 2.4);

  Yaw_trimx = constrain(Yaw_trim,-2.6, 2.6);
  setpoint = targetAngle + Pitch_trim - OutputPx;
  error = currentAngle - setpoint;   
  errorSum += error;  
  errorSum = constrain(errorSum, -400, 400);
  Output = Kp*(error) + Ki*(errorSum)*T + Kd*(currentAngle-prevAngle)/T;;
  prevAngle = currentAngle;
  
  ///ERROR YAW
  setpointYaw = (targetYaw - Yaw_trim - AngleZ); 
  currentAngleY = AngleZ;
  errorY = currentAngleY - setpointYaw;
  errorSumY += errorY;  
  errorSumY = constrain(errorSumY, -400, 400);
  OutputY = Kpy*(errorY) + Kiy*(errorSumY)*T + Kdy*(currentAngleY-prevAngleY)/T;
  prevAngleY = currentAngleY;
  targetYaw = currentAngleY;

  //////----Motor Control----///////

}
void Motor(){
  //lIMIT OUTPUT
  Output = constrain(Output, -Output_limit, 255);
  OutputP = constrain(OutputP, -40, 40); 
  OutputY = constrain(OutputY, -200, 200);

  speedLeft = Output + OutputP - OutputY; // - OutputY 
  speedRight = Output + OutputP + OutputY ;// + OutputY 
  speedLeft = constrain(speedLeft, -255, 255);
  speedRight = constrain(speedRight, -255, 255);

  if ((speedLeft > 0) && (speedRight > 0)) { // (speedLeft > 0) && (speedRight > 0)
    // Set the motors to move forward
    digitalWrite(DIR1L, HIGH);
    digitalWrite(DIR2L, LOW);
    digitalWrite(DIR1R, HIGH);
    digitalWrite(DIR2R, LOW);

    analogWrite(PWMR, speedRight);
    analogWrite(PWML, speedLeft);

  } else if ((speedLeft < 0) && (speedRight < 0)) { // (speedLeft < 0) && (speedRight < 0)
    // Set the motors to move in reverse
    digitalWrite(DIR1L, LOW);
    digitalWrite(DIR2L, HIGH);
    digitalWrite(DIR1R, LOW);
    digitalWrite(DIR2R, HIGH);

    analogWrite(PWMR, abs(speedRight));
    analogWrite(PWML, abs(speedRight));

  }  else {
    // Stop the motors
    analogWrite(PWMR, 0);
    analogWrite(PWML, 0);
  }
}

void getAngle(){
  
  // CALCULATE AND FILTER 
  accAngle = atan2(ax, -az)*RAD_TO_DEG;
  gyroRate = map(gyroY, -32768, 32767, -200, 200);
  gyroAngle = (float)gyroRate*T;  
  currentAngle = alpha*(prevAngle + gyroAngle) + (1-alpha)*(accAngle); 

}

void Pulse_Pitch()
{
  if (micros() > time_Star)
  {
    xung = micros() - time_Star;
    time_Star = micros();
    if(xung <2000 && xung > 1000) Pitch = bo_loc.updateEstimate(xung);
    Pitch_trim = map(Pitch, 1100, 1900, -10, 10);
  }
}

void Pulse_Yaw()
{
  if (micros() > time_Star1)
  {
    xung1 = micros() - time_Star1;
    time_Star1 = micros();
    if(xung1 <2000){
      //Yaw = kalman(xung1);
      filteredValue = ALPHA * filteredValue + (1 - ALPHA) * xung1;
    }
    Yaw_trim = map(filteredValue, 1200,1800, -19, 19);
    
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

double kalman(double U) {                                                     // Bộ lọc nhiễu tín hiệu cảm biến siêu âm
  static const double R = 1;
  static const double H = 1.00;
  static double Q = 0.008;
  static double P = 0.0;
  static double U_hat = 0;
  static double K = 0;
  K = P * H / (H * P * H + R);
  U_hat += +K * (U - H * U_hat);
  P = (1 - K * H) * P + Q;
  
  return U_hat;
}
