#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <utility/imumaths.h>
#include <string.h>

// BNO055 sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55);
// GPS object 
SFE_UBLOX_GNSS GPS;
// BMP388 sensor object
Adafruit_BMP3xx bmp = Adafruit_BMP388(88);
// Declare global variables and constants 
unsigned long startTime; 
unsigned long currentTime; 
int totalPackets = 1; 
String teamID = "BillyBob";
void setup() {
  
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial1) delay(10);  // wait for serial port to open!
  
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while (1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  delay(1000);

//GPS


}



void loop() {
  startTime = millis();
  //collect and output absolute orientation by euler angle
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  //collect and output angular velocity with the gyro
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Collect and output acceleration
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  //collect and output temperature
  int temp = bno.getTemp();

  // gather and output GPS data
 
  if(millis() - startTime < 50) {
    delay(50 - (millis() - startTime));
  }
}


