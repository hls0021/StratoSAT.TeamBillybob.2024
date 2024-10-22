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
Adafruit_BMP3XX bmp;
// Declare global variables and constants 
unsigned long startTime; 
unsigned long currentTime; 
int totalPackets = 1; 
String teamID = "BillyBob";
void setup() {
  
  Serial.begin(115200);
  Serial5.begin(115200);
  while (!Serial5) delay(10);  // wait for serial port to open!
  
  if (!bno.begin())
  {
    Serial5.print("No BNO055 detected");
    while (1);
  }
  if (!bmp.begin_I2C()) {  
    Serial5.println("BMP388 initialization failed!");
    while (1);  
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  delay(1000);

    //GPS
    if (GPS.begin()) {
        Serial5.println("u-blox GNSS module initialized successfully!");
    } else {
        Serial5.println("u-blox GNSS module initialization failed!");
        while (1);  // If GNSS doesn't initialize, stop the program
    }
    //Start Time
      startTime = millis();
}



void loop() {
    startTime = millis();
  //collect and output absolute orientation by euler angle
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        Serial5.print("Orientation (Euler angles): ");
        Serial5.print("Heading: "); Serial5.print(euler.x());
        Serial5.print(" Roll: "); Serial5.print(euler.y());
        Serial5.print(" Pitch: "); Serial5.println(euler.z());
  //collect and output angular velocity with the gyro
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        Serial5.print("Angular Velocity (Gyro): ");
        Serial5.print(gyro.x()); Serial5.print(", ");
        Serial5.print(gyro.y()); Serial5.print(", ");
        Serial5.print(gyro.z()); Serial5.println(" rad/s");
  // Collect and output acceleration
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        Serial5.print("Acceleration: ");
        Serial5.print(accel.x()); Serial5.print(", ");
        Serial5.print(accel.y()); Serial5.print(", ");
        Serial5.print(accel.z()); Serial5.println(" m/s^2");

  //collect and output temperature 
  if (!bmp.performReading()) {
        Serial5.println("Failed to perform BMP388 reading");
        return;
        }
        Serial5.print("BMP388 Temperature: ");
        Serial5.print(bmp.readTemperature());
        Serial5.println(" °C");
        Serial5.print("BMP388 Altitude: ");
        Serial5.print(bmp.readAltitude(630));  // Adjust sea level pressure need to fix
        Serial5.println(" m");
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
        Serial5.println(" m");

        }
       
   // Gather and output time spent and UTC time
       Serial.println();
    Serial.print(GPS.getYear());
    Serial.print("-");
    Serial.print(GPS.getMonth());
    Serial.print("-");
    Serial.print(GPS.getDay());
    Serial.print(" ");
    Serial.print(GPS.getHour());
    Serial.print(":");
    Serial.print(GPS.getMinute());
    Serial.print(":");
    Serial.print(GPS.getSecond());


  if(millis() - startTime < 50) {
    delay(50 - (millis() - startTime));
  }
}


