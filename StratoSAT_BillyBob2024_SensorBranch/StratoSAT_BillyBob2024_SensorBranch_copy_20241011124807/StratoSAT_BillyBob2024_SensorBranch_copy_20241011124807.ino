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
Adafruit_BMP3XX bmp = Adafruit_BMP3XX(88);
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
    if (GPS.begin()) {
        Serial.println("u-blox GNSS module initialized successfully!");
    } else {
        Serial.println("u-blox GNSS module initialization failed!");
        while (1);  // If GNSS doesn't initialize, stop the program
    }
    //Start Time
      startTime = millis();
}



void loop() {
  //collect and output absolute orientation by euler angle
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        Serial1.print("Orientation (Euler angles): ");
        Serial1.print("Heading: "); Serial1.print(euler.x());
        Serial1.print(" Roll: "); Serial1.print(euler.y());
        Serial1.print(" Pitch: "); Serial1.println(euler.z());
  //collect and output angular velocity with the gyro
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        Serial1.print("Angular Velocity (Gyro): ");
        Serial1.print(gyro.x()); Serial1.print(", ");
        Serial1.print(gyro.y()); Serial1.print(", ");
        Serial1.print(gyro.z()); Serial1.println(" rad/s");
  // Collect and output acceleration
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        Serial1.print("Acceleration: ");
        Serial1.print(accel.x()); Serial1.print(", ");
        Serial1.print(accel.y()); Serial1.print(", ");
        Serial1.print(accel.z()); Serial1.println(" m/s^2");

  //collect and output temperature
  int temp = bno.getTemp();

  // Collect and output GPS data 
        if (GPS.getLatitude() != 0 && GPS.getLongitude() != 0) {
            Serial5.print("GPS Location: ");
            Serial5.print(GPS.getLatitude() / 10000000.0, 6);  // Latitude
            Serial5.print(", ");
            Serial5.print(GPS.getLongitude() / 10000000.0, 6);  // Longitude
            Serial5.println();
        }

        if (GPS.getAltitude() != 0) {
            Serial5.print("Altitude: ");
            Serial5.print(GPS.getAltitude() / 1000.0);  // Altitude in meters
            Serial1.println(" m");
        }
       

  //Record Time 
          // Gather and output time spent and UTC time
        currentTime = millis();
        unsigned long timeSpent = currentTime - startTime;
        Serial1.print("Time spent since start: ");
        Serial1.print(timeSpent / 1000);  // in seconds
        Serial1.println(" seconds");
 
  if(millis() - startTime < 50) {
    delay(50 - (millis() - startTime));
  }
}


