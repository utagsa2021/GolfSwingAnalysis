#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>


#define BNO055_SAMPLERATE_DELAY_MS (50)

Adafruit_BNO055 myIMU = Adafruit_BNO055();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  myIMU.begin();
  delay(1000);
  int8_t temp=myIMU.getTemp();
  myIMU.setExtCrystalUse(true);
//  Serial.print(temp);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t system, gyro, accel, mg = 0;
  myIMU.getCalibration(&system, &gyro, &accel, &mg);
  imu::Vector<3> lin_acc =myIMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  
  imu::Quaternion quat=myIMU.getQuat();
  
  Serial.print(accel);
  Serial.print(",");
  Serial.print(gyro);
  Serial.print(",");
  Serial.print(mg);
  Serial.print(",");
  Serial.print(system);
  Serial.print(",");  
  Serial.print(quat.w());
  Serial.print(",");
  Serial.print(quat.x());
  Serial.print(",");
  Serial.print(quat.y());
  Serial.print(",");
  Serial.print(quat.z());
  Serial.print(",");
  Serial.print(lin_acc.x());
  Serial.print(",");
  Serial.print(lin_acc.y());
  Serial.print(",");
  Serial.print(lin_acc.z());
  Serial.print(",");
  Serial.println(millis());

  
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
