#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
<<<<<<< HEAD
#include <SparkFun_u-Blox_NEO_M9N.h>
#include <SAMD21_Dev_Breakout.h>
=======
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h>
>>>>>>> 1907a82c7de502d894e4841595df6784b2773f9a

// BNO055 sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55);
// GPS object 
Adafruit_GPS GPS(&Serial1);
// Declare global variables and constants 
unsigned long startTime; 
unsigned long currentTime; 
int totalPackets = 1; 
const char* teamID = "BillyBob"; //Subject to change
void setup() {
  
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!
  
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while (1);
  }

  delay(1000);

	//serial communication
	Serial.begin(9600);
	Serial1.begin(9600);
//GPS

// Initialize SD card
    if (!SD.begin(chipSelect)) {
        Serial.println("SD card initialization failed!");
        while (1);
    }
    Serial.println("SD card initialized.");

// Open file to write data
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
        dataFile.println("Payload Data Logging Started");
        dataFile.println("Team ID: " + String(teamID));
        dataFile.close();
    }

}



void loop() {

delay(50)
//collect and output absolute orientation by euler angle
imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

 //collect and output angular velocity with the gyro
imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

// Collect and output acceleration
imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

//collect and output temperature
int = bno.getTemp()

// gather and output GPS data
?????????
 
}

//log data
dataFile = SD.open("datalog.txt", FILE_WRITE);
        if (dataFile) {
           // Output Euler angles (Orientation)
            dataFile.print("Orientation (Euler angles): ");
            // Output Gyroscope data
            dataFile.print("Angular Velocity (Gyro): ");
            // Output Acceleration
            dataFile.print("Acceleration: ");
            // Output Temperature
            dataFile.print("Temperature: ");
            // Output GPS data
            dataFile.print("GPS Location: ");
            // Output Time spent
            dataFile.print("Time spent since start: ");
            dataFile.print("UTC Time: ");
            } else {
            Serial.println("Error opening datalog.txt");
        }

