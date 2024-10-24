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
float seaLevel;
void setup() {
  
  Serial.begin(9600);
  Serial.println("Serial connected!");
  Serial5.begin(9600);
  while (!Serial5) delay(10);  // wait for serial port to open!
  
  while (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    delay(1000);
  }
  Serial.println("BNO Success");
  while (!bmp.begin_I2C()) {  
    Serial.println("BMP388 initialization failed!");
    delay(1000);  
  }
  Serial.println("BMP Success");
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

  bmp.readPressure();
  delay(10);
  bmp.readPressure();
  delay(10);
  seaLevel = bmp.pressure / 100.0;

    //Start Time
      startTime = millis();
}



void loop() {
    startTime = millis();
  //collect and output absolute orientation by euler angle
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        Serial.print("Orientation (Euler angles): ");
        Serial.print("Heading: "); Serial.print(euler.x());
        Serial.print(" Roll: "); Serial.print(euler.y());
        Serial.print(" Pitch: "); Serial.println(euler.z());
  //collect and output angular velocity with the gyro
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        Serial.print("Angular Velocity (Gyro): ");
        Serial.print(gyro.x()); Serial.print(", ");
        Serial.print(gyro.y()); Serial.print(", ");
        Serial.print(gyro.z()); Serial.println(" rad/s");
  // Collect and output acceleration
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        Serial.print("Acceleration: ");
        Serial.print(accel.x()); Serial.print(", ");
        Serial.print(accel.y()); Serial.print(", ");
        Serial.print(accel.z()); Serial.println(" m/s^2");

  //collect and output temperature 
  if (!bmp.performReading()) {
        Serial.println("Failed to perform BMP388 reading");
        return;
        }
        Serial.print("BMP388 Temperature: ");
        Serial.print(bmp.readTemperature());
        Serial.println(" °C");
        Serial.print("BMP388 Altitude: ");
        Serial.print(bmp.readAltitude(seaLevel));  // Adjust sea level pressure need to fix
        Serial.println(" m");
  // Collect and output GPS data 
  if (GPS.getLatitude() != 0 && GPS.getLongitude() != 0) {
        Serial.print("GPS Location: ");
        Serial.print(GPS.getLatitude() / 10000000.0, 6);  // Latitude
        Serial.print(", ");
        Serial.print(GPS.getLongitude() / 10000000.0, 6);  // Longitude
        Serial.println();
        }

  if (GPS.getAltitude() != 0) {
        Serial.print("Altitude: ");
        Serial.print(GPS.getAltitude() / 1000.0);  // Altitude in meters
        Serial.println(" m");

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


