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
float waitingTime;
float altitude;
float previous1altitude;
float previous2altitude;
float previous3altitude;
float previous4altitude;
float previous5altitude;
float previous6altitude;
float previous7altitude;
float previous8altitude;
float previous9altitude;
float previous10altitude;
float previous11altitude;
float previous12altitude;
float previous13altitude;
float previous14altitude;
float previous15altitude;
float previous16altitude;
float previous17altitude;
float previous18altitude;
float previous19altitude;
float previous20altitude;
float previous21altitude;
float previous22altitude;
float previous23altitude;
float previous24altitude;
float previous25altitude;
float previous26altitude;
float previous27altitude;
float previous28altitude;
float previous29altitude;
float previous30altitude;
float previous31altitude;
float previous32altitude;
float previous33altitude;
float previous34altitude;
float previous35altitude;
float previous36altitude;
float previous37altitude;
float previous38altitude;
float previous39altitude;
float previous40altitude;
float gyro;
float ledonoff = 0;
float Kp = 0;
float Ki = 0;
float Kd = 0;
float integral;
float derivative;
float reference;
float lasterror = 0;
float error;
sensors_event_t event;
int solenoidclock = 14;
int solenoidcounter = 15;
float output;
imu::Vector<3> angularvelocity;
imu::Vector<3> orientation;
imu::Vector<3> acceleration;
float steady;
float packetcount;
float seaLevel;
//TODO angularVelocity and orientation need to be type vector
// imu::Vector<3> orientation;


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
    if(acceleration.x() < 0) {}
      currentState = DESCENT;
    }
}


//descent stae
void descent() {
  if (altitude <= previous40altitude + 15000) {
    waitingTime = waitingTime + startTime - endTime;
    endTime = startTime - preTime;
    if (waitingTime >= 420000) {
        currentState = LANDING;
    }
  }
}

void landing() {
  while(1);
}

void setup() {
  
  Serial5.begin(9600);
  Serial5.println("Serial connected!");
  Serial5.begin(9600);
  while (!Serial5) delay(10);  // wait for serial port to open!
  
  while (!bno.begin())
  {
    Serial5.print("No BNO055 detected");
    tenloops = tenloops + 2;
  if (tenloops >= 10) {
    if(ledonoff == 0) {
    digitalWrite(led, HIGH);
    tenloops = tenloops - 10;
    ledonoff = 1;
    }
    else {
    if(ledonoff == 0) {
      digitalWrite(led, LOW);
      tenloops = tenloops - 10;
      ledonoff = 0;
      }
    }
  }
    delay(1000);
  }
  Serial5.println("BNO Success");
  while (!bmp.begin_I2C()) {  
    Serial5.println("BMP388 initialization failed!");
      tenloops = tenloops + 4;
  if (tenloops >= 10) {
    if(ledonoff == 0) {
    digitalWrite(led, HIGH);
    tenloops = tenloops - 10;
    ledonoff = 1;
    }
    else {
    if(ledonoff == 0) {
      digitalWrite(led, LOW);
      tenloops = tenloops - 10;
      ledonoff = 0;
      }
    }
  }
    delay(50);  
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

  bmp.readPressure();
  delay(10);
  bmp.readPressure();
  delay(10);
  bmp.readPressure();
  delay(10);
  bmp.readPressure();
  delay(10);
  seaLevel = bmp.pressure / 100.0 + 193000;

  Serial5.begin(9600); 
  pinMode(led, OUTPUT);
  Serial5.print("Team_ID,");
  Serial5.print("Mission Time,");
  Serial5.print("Packet Count,");
  Serial5.print("Accleration,");
  Serial5.print("Eular X,");
  Serial5.print("Eular Y,");
  Serial5.print("Eular Z,");
  Serial5.print("Gyro X,");
  Serial5.print("Gyro Y,");
  Serial5.print("Gyro Z,");
  Serial5.print("Pressure ALT,");
  Serial5.print("Temperature,");
  Serial5.print("GPS LONG,");
  Serial5.print("GPS LAT,");
  Serial5.print("GPS ALT,");
  Serial5.print("UTC Time");
  Serial5.println("Software State");
}



void loop() {
  startTime = millis();
  Serial5.print("BILLYBOB");
  Serial5.print(startTime);
  Serial5.print(packetcount);
  //collect and output absolute orientation by euler angle
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        Serial5.print("Heading: "); Serial5.print(euler.x());
        Serial5.print(" Roll: "); Serial5.print(euler.y());
        Serial5.print(" Pitch: "); Serial5.println(euler.z());
  //collect and output angular velocity with the gyro
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        Serial5.print(gyro.x()); Serial5.print(", ");
        Serial5.print(gyro.y()); Serial5.print(", ");
        Serial5.print(gyro.z()); Serial5.println(" rad/s");
  // Collect and output acceleration
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        Serial5.print(accel.x()); Serial5.print(", ");
        Serial5.print(accel.y()); Serial5.print(", ");
        Serial5.print(accel.z()); Serial5.println(" m/s^2");

  //collect and output temperature 
  if (!bmp.performReading()) {
        Serial5.println("Failed to perform BMP388 reading");
        return;
        }
        Serial5.print(bmp.readTemperature());
        Serial5.print(bmp.readAltitude(seaLevel));  // Adjust sea level pressure need to fix
  // Collect and output GPS data 
  if (GPS.getLatitude() != 0 && GPS.getLongitude() != 0) {
        Serial5.print(GPS.getLatitude() / 10000000.0, 6);  // Latitude
        Serial5.print(GPS.getLongitude() / 10000000.0, 6);  // Longitude
        }

  if (GPS.getAltitude() != 0) {
        Serial5.print(GPS.getAltitude() / 1000.0);  // Altitude in meters

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
    Serial5.println(GPS.getSecond());

  //Proportional term
  bno.getEvent(&event);
  angularvelocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  altitude = GPS.getAltitude(seaLevel) / 1000;


  if(angularvelocity.x() >= 10) {
    digitalWrite(solenoidclock, HIGH);
    digitalWrite(solenoidcounter, LOW);
  }
  if(angularvelocity.x() <= -10) {
    digitalWrite(solenoidclock, LOW);
    digitalWrite(solenoidcounter, HIGH);
  }
  error = reference - orientation.x();
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
    if(ledonoff == 0) {
    digitalWrite(led, HIGH);
    tenloops = tenloops - 10;
    ledonoff = 1;
    }
    else {
    if(ledonoff == 0) {
      digitalWrite(led, LOW);
      tenloops = tenloops - 10;
      ledonoff = 0;
      }
    }
  }
  
  switch (currentState) {
  //launchReady(), ascend(), stabilization(), descent(), and landing() are functions that are declared after 
    case LAUNCH_READY:
      launchReady();
      Serial5.print("Launch Ready");
      break;
    case ASCEND:
      ascend();
      Serial5.print("Ascend");
      break;
    case STABILIZATION:
      stabilization();
      Serial5.print("Stabilization");
      break;
    case DESCENT:
      descent();
      Serial5.print("Descent");
      break;
    case LANDING:
      landing();
      Serial5.print("Landing");
      break;
  
  if(millis() - startTime < 50) {
    delay(50 - (millis() - startTime));

    //determine height from 40 runs ago
    previous40altitude = previous39altitude;
    previous39altitude = previous38altitude;
    previous38altitude = previous37altitude;
    previous37altitude = previous36altitude;
    previous36altitude = previous35altitude;
    previous35altitude = previous34altitude;
    previous34altitude = previous33altitude;
    previous33altitude = previous32altitude;
    previous32altitude = previous31altitude;
    previous31altitude = previous30altitude;
    previous30altitude = previous29altitude;
    previous29altitude = previous28altitude;
    previous28altitude = previous27altitude;
    previous27altitude = previous26altitude;
    previous26altitude = previous25altitude;
    previous25altitude = previous24altitude;
    previous24altitude = previous23altitude;
    previous23altitude = previous22altitude;
    previous22altitude = previous21altitude;
    previous21altitude = previous20altitude;
    previous20altitude = previous19altitude;
    previous19altitude = previous18altitude;
    previous18altitude = previous17altitude;
    previous17altitude = previous16altitude;
    previous16altitude = previous15altitude;
    previous15altitude = previous14altitude;
    previous14altitude = previous13altitude;
    previous13altitude = previous12altitude;
    previous12altitude = previous11altitude;
    previous11altitude = previous10altitude;
    previous10altitude = previous9altitude;
    previous9altitude = previous8altitude;
    previous8altitude = previous7altitude;
    previous7altitude = previous6altitude;
    previous6altitude = previous5altitude;
    previous5altitude = previous4altitude;
    previous4altitude = previous3altitude;
    previous3altitude = previous2altitude;
    previous2altitude = previous1altitude;
    previous1altitude = altitude;
    }
  }
}
