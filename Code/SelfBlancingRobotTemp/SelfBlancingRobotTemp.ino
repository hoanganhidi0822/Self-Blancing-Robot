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

/* float Kp = 150;                      
float Kd = 0.2; 
float Ki = 300;  */ 
float Kp = 60;   //21.4                   
float Kd = 0.55; //0.0285    verygood p32.1 d0.128 i150 //0.5kd
float Ki = 0; //84
float  targetAngle = 5;
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

int xungL, xungR, tam;
volatile float goc, goc_fitered;

int Output;

void setup() {

  TCCR2B = TCCR2B & B11111000 | B00000001;  //PWM frequency of 31372.55 Hz
  
  //TCCR2B = TCCR2B & B11111000 | B00000010;//3921.15hz
  //TCCR2A = (TCCR2A & B11111100) | B00000011;

/*   OCR2A = 78; 
  OCR2B = 0; */

  /* pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);

  // Configure Timer 2 for Fast PWM mode
  TCCR2A = (1 << WGM21) | (1 << WGM20);  // Fast PWM mode
  TCCR2B = (1 << CS22);  // Clock prescaler of 64

  // Set the initial PWM values (0% duty cycle)
  OCR2A = 0;  // For pin 11
  OCR2B = 0;  // For pin 3 */

  pinMode(2,INPUT_PULLUP);
  //pinMode(3,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  pinMode(5,INPUT_PULLUP);

  pinMode(DIR1L, OUTPUT);
  pinMode(DIR2L, OUTPUT);
  pinMode(DIR1R, OUTPUT);
  pinMode(DIR2R, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);

  Wire.begin();
  mpu.initialize();
  Serial.begin(9600);

  //attachInterrupt(0,DemxungL,FALLING);
  //attachInterrupt(digitalPinToInterrupt(10),DemxungR,FALLING);
  
  Timer1.initialize(7000);  //don vi us
  Timer1.attachInterrupt(PID);

}

void loop() {
  //mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  ax = mpu.getAccelerationX();
  az = mpu.getAccelerationZ(); 
  //gyroY = mpu.getRotationX();
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);
  
  //Serial.print(accAngle);
  delay(7);
  Serial.print(gyroY);
  Serial.print("  ");
  Serial.println(Output);
  
  
  
  
}


void PID()
{
  //goc = atan2(ax, az) * RAD_TO_DEG;

  accAngle = atan2(ax, -az)*RAD_TO_DEG;
 /*  if (accAngle > 0 && accAngle < 180){
    accAngle = 180 - accAngle;
  }
  else if (accAngle < 0 && accAngle > -180){
    accAngle =  accAngle + 180;
    accAngle = -accAngle;
  }else{
    accAngle = 0;
  } */
  gyroRate = map(gyroY, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*T;  
  //gyroAngle =gyroAngle + (float)gyroRate*T/1000;
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle); 
  goc = currentAngle;
  
  
   //goc_fitered = kalman(goc);
   goc_fitered = goc;

//////////////////////////////
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -350, 350);
  //calculate output from P, I and D values
  Output = Kp*(error) + Ki*(errorSum)*T + Kd*(currentAngle-prevAngle)/T;
  tam = Kd*(currentAngle-prevAngle)/T;
  prevAngle = currentAngle;

  
//////////////////////////////
  
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
  if(digitalRead(5) == LOW)
  xungR ++;
  else
  xungR --;
}

double kalman(double U) {                                                     // Bộ lọc nhiễu tín hiệu cảm biến siêu âm
  static const double R = 40;
  static const double H = 1.00;
  static double Q = 10;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;
  K = P * H / (H * P * H + R);
  U_hat += +K * (U - H * U_hat);
  P = (1 - K * H) * P + Q;
  return U_hat;
}


