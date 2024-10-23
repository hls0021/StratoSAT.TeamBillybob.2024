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

float tenloops;
float preTime;
float endTime;
float altitude;
float gyro;
float ledonoff = 0;
float Kp;
float Ki;
float Kd;
float integral;
float derivative;
float reference;
float lasterror = 0;
float error;
sensor_event_t event;
int solenoidclock = 14;
int solenoidcounter = 15;
float output;
float angularvelocity;
float orientation;
float steady = orientation.x();


//led
int led = 16;

//state definition
enum FlightState {
  LAUNCH_READY,
  ASCEND,
  STABILIZATION,
  DESCENT,
  LANDING
};
FlightState currentState = LAUNCH_READY;

void setup() {
  
  Serial.begin(9600);
  Serial5.println("Serial connected!");
  Serial5.begin(9600);
  while (!Serial5) delay(10);  // wait for serial port to open!
  
  while (!bno.begin())
  {
    Serial5.print("No BNO055 detected");
    delay(1000);
  }
  Serial5.println("BNO Success");
  while (!bmp.begin_I2C()) {  
    Serial5.println("BMP388 initialization failed!");
    delay(1000);  
  }
  Serial5.println("BMP Success");
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

  Serial.begin(9600); 
  pinMode(led, OUTPUT);

}



void loop() {
  startTime = millis();
  //Proportional term
  bno.getevent(&event);
  angularvelocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  orienation = bno.getVector(Adafruit_BNO055::VECTOR_EULER); 
  steady = orientation.x();

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
       Serial5.println();
    Serial5.print(GPS.getYear());
    Serial5.print("-");
    Serial5.print(GPS.getMonth());
    Serial5.print("-");
    Serial5.print(GPS.getDay());
    Serial5.print(" ");
    Serial5.print(GPS.getHour());
    Serial5.print(":");
    Serial5.print(GPS.getMinute());
    Serial5.print(":");
    Serial5.print(GPS.getSecond());

  if(angularvelocity >= 10) {
    digitalWrite(solenoidclock, HIGH);
    digitalWrite(solenoidcounter, LOW);
  }
  if(angularvelocity <= -10) {
    digitalWrite(solenoidclock, LOW);
    digitalWrite(solenoidcounter, HIGH);
  }
  error = reference - steady;
  //Integral and Derivative terms 
  integral = integral + error * (50 - (millis() - startTime));
  derivative = (error - lasterror) / (50 - (millis() - startTime));
  //Output
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  lasterror = error;

  if(output >= 15) {
    digitalWrite(solenoidclock, HIGH);
    digitalWrite(solenoidcounter, HIGH);
  }
  if(output <= -15) {
    digitalWrite(solenoidclock, LOW);
    digitalWrite(solenoidcounter, HIGH);
  }  
  if(output >= 2) {
    if(output <= 2) {
      digitalWrite(solenoidclock, LOW);
      digitalWrite(solenoidcounter, LOW);
    }
  }


    //state transition
  tenloops = tenloops + 1;
  if (tenloops >= 10) {
    if(ledonoff = 0) {
    digitalWrite(led, HIGH);
    tenloops = tenloops - 10;
    ledonoff = 1;
    }
    else {
    if(ledonoff = 0) {
      digitalWrite(led, LOW);
      tenloops = tenloops - 10;
      ledonoff = 0;
      }
    }
  }
  
  switch (currentState) {

    case LAUNCH_READY:
    launchReady();
    break;

    case ASCEND:
    ascend();
    break;
    case STABILIZATION:
    stabilization();
    break;
    case DESCENT:
    descent();
    break;
    case LANDING:
    landing();
    break;

    if(millis() - startTime < 50) {
    delay(50 - (millis() - startTime));
  }
}

//launch ready state
void launchReady() {
  if (altitude > 100000) {
    currentState = ASCEND;
  }
}

//ascend state
void ascend() {
  if (altitude > 1600000) {
    currentState = STABILIZATION;
  }
}

//stabilization state
void stabilization() {
  if (altitude > 2700000) {
    if(acceleration < 0)
    currentState = DESCENT;
  }
}

//descent stae
void descent() {
  if (altitude <= 400000) {
    if (gyro = 0) {
      waitingTime = waitingTime + startTime - endTime;
      endTime = startTime - preTime;
      if (waitingTime >= 420000) {
        currentState = LANDING;
      }
    }
  }
}

void landing() {
  break;
}