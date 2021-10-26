#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

long timer = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin(2, 3);
  // stop everything if could not connect to MPU6050
  while(status!=0){ }
  
  mpu.setFilterGyroCoef(.9);
  
  // gyro and accelero
  mpu.calcOffsets(true,true); 
  Serial.println("Done!\n");
  
}

void loop() {
  //  Serial.print(map(mpu.getAngleX(), 1000, -1000, 0, 360));
  //  Serial.print(map(mpu.getAngleZ(), -1000, 1000, 0, 360));
  //  Serial.print(map(mpu.getAngleZ(), -1000, 1000, 0, 360));

  mpu.update();
  // X axis
  Serial.print(mpu.getAngleX());
  Serial.print("|");
  // Y axis
  Serial.print(mpu.getAngleY());
  Serial.print("|");
  // Z axis
  Serial.print(mpu.getAngleZ());
  Serial.println("");
  // Serial.write?
  delay(10);
}
