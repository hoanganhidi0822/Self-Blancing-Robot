
#include <MPU6050_tockn.h>
#include <Wire.h>
MPU6050 mpu6050(Wire);

int x, y, z;
int den = 5;

void setup() 
{
  Serial.begin(9600);
  
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  pinMode(den,OUTPUT);
  digitalWrite(den, LOW);
}

void loop() 
{
  mpu6050.update();
  y = mpu6050.getAngleY();
/*   if (y >= 0){
    y = map(y, 180, 0, 0, 180);
  }
  if (y < 0){
    y = map(y, -180, 0, 0, -180);
  } */
  
  Serial.print("Y: "); Serial.println(y);
  delay(10);
  
}
