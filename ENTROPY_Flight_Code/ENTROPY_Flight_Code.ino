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
unsigned long prevTime = 0; 
unsigned long currentTime; 
unsigned long GPS_Tick = 500;
unsigned long GPS_prevTime = 0;

float tenloops = 0;
float preTime;
float endTime;
float waitingTime;
float altitude;
/*float previous1altitude;
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
float previous40altitude;*/
//float gyro;
float ledonoff = 0;
float Kp = 2;
float Ki = 0.3;
float Kd = 0.2;
float integral;
float derivative;
float reference;
float lasterror = 0;
float error;
sensors_event_t event;
int solenoidclock = 14;
int solenoidcounter = 15;
float output;
imu::Vector<3> angularvelocity; // Was Commented Out
imu::Vector<3> orientation;
imu::Vector<3> gyro;
imu::Vector<3> acceleration;
float steady;
float packetcount = 0;
float seaLevel = 0;
//TODO angularVelocity and orientation need to be type vector
// imu::Vector<3> orientation;

// Data Collection Variables ------------------------------------- 
unsigned long Time;
int totalPackets = 1; 
String teamID = "BillyBob";
float acceleration_x;
float acceleration_y;
float acceleration_z;
float x_orientation;
float y_orientation;
float z_orientation;
float gyro_x;
float gyro_y;
float gyro_z;
float Pressure_ALT = 0;
float temp;
double GPS_LONG = 0;
double GPS_LAT = 0;
long GPS_Altitude = 0;
String Unix_Time = " ";
String software_state;

const String stateNames[5] {"LAUNCH_READY", "ASCEND", "STABILIZATION", "DESCENT", "LANDING"};
//-----------------------------------------------------------------

//led
int led = 16;

//state definition
enum FlightState {
  LAUNCH_READY = 0,
  ASCEND = 1,
  STABILIZATION = 2,
  DESCENT = 3,
  LANDING = 4
};
FlightState currentState = STABILIZATION;

//launch ready state
void launchReady() {
  if (Pressure_ALT > 100000) {
    currentState = ASCEND;
  }
}

//ascend state
void ascend() {
  if (Pressure_ALT > 1600000) {
    currentState = STABILIZATION;
  }
}

//stabilization state
void stabilization() {
  if(gyro.y() >= 10) {
    digitalWrite(solenoidclock, HIGH);
    digitalWrite(solenoidcounter, LOW);
  }
  else if(gyro.y() <= -10) {
    digitalWrite(solenoidclock, LOW);
    digitalWrite(solenoidcounter, HIGH);
  }
  else if(gyro.y() >= -10 && gyro.y() <= 10) {
    digitalWrite(solenoidclock, HIGH);
    digitalWrite(solenoidcounter, HIGH);
  }
  /* error = reference - orientation.x();
  //Integral and Derivative terms 
  integral = integral + error * (50 - (millis() - startTime));
  derivative = (error - lasterror) / (50 - (millis() - startTime));
  //Output
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  lasterror = error;

  if(output >= 15) {
    digitalWrite(solenoidclock, HIGH);
    digitalWrite(solenoidcounter, LOW);
  }
  if(output <= -15) {
    digitalWrite(solenoidclock, LOW);
    digitalWrite(solenoidcounter, HIGH);
  }  
  if(output >= 2) {
    if(output <= 2) {
      digitalWrite(solenoidclock, LOW);
      digitalWrite(solenoidcounter, LOW);
    }*/
  if (Pressure_ALT > 2400000) {
    if(acceleration_x < 0) {
      currentState = DESCENT;
      }
    }
  }


//descent stae
void descent() {
  digitalWrite(solenoidclock, LOW);
  digitalWrite(solenoidcounter, LOW);
  /*if (altitude <= previous40altitude + 15000 && altitude >= previous40altitude - 15000) {
    endTime = startTime - preTime;
    waitingTime = waitingTime + startTime - endTime;
    if (waitingTime >= 420000) {
        currentState = LANDING;
    }
  }*/
}

void landing() {

}

void getDataFromSensors() {

  orientation = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  Time = millis();
  teamID = "BillyBob";
  acceleration_x = acceleration.x();
  acceleration_y = acceleration.y();
  acceleration_z = acceleration.z();
  x_orientation = orientation.x();
  y_orientation = orientation.y();
  z_orientation = orientation.z();
  gyro_x = gyro.x();
  gyro_y = gyro.y();
  gyro_z = gyro.z();
  Pressure_ALT = bmp.readAltitude(seaLevel);
  temp = bmp.temperature;
  ////////////////////////
  /*if (GPS.checkUblox() && Time - GPS_prevTime > GPS_Tick) //was GPSTick changed to GPS_Tick
  {
   GPS_LONG = GPS.getLongitude();
   GPS_LAT = GPS.getLatitude();
   GPS_Altitude = GPS.getAltitude();
   Unix_Time = GPS.getUnixEpoch();
   GPS_prevTime = Time;
  } */
  //////////////////////////
  software_state = stateNames[currentState];

}

void writeDataToSD() {
  Serial5.print("BillyBob,");
  Serial5.print(Time);
  Serial5.print(",");
  Serial5.print(totalPackets);
  Serial5.print(",");
  Serial5.print(acceleration_x);
  Serial5.print(",");
  Serial5.print(acceleration_y);
  Serial5.print(",");
  Serial5.print(acceleration_z);
  Serial5.print(",");
  Serial5.print(x_orientation);
  Serial5.print(",");
  Serial5.print(y_orientation);
  Serial5.print(",");
  Serial5.print(z_orientation);
  Serial5.print(",");
  Serial5.print(gyro_x);
  Serial5.print(",");
  Serial5.print(gyro_y);
  Serial5.print(",");
  Serial5.print(gyro_z);
  Serial5.print(",");
  Serial5.print(Pressure_ALT);
  Serial5.print(",");
  Serial5.print(temp);
  Serial5.print(",");
  Serial5.print(software_state);
  Serial5.print("\n");

  totalPackets = totalPackets + 1;
}

void setup() {
  Time = millis();
  Serial5.begin(9600);
  Serial5.println("Serial5 connected!");

  pinMode(solenoidclock, OUTPUT);
  pinMode(solenoidcounter, OUTPUT);
  pinMode(led, OUTPUT);
  

  while (!Serial5) delay(10);  // wait for Serial5 port to open!
  
  while (!bno.begin())
  {
    Serial5.print("No BNO055 detected");
    digitalWrite(led, HIGH);
    delay(1000);
  }
  Serial5.println("BNO Success");
  while (!bmp.begin_I2C()) {  
    Serial5.println("BMP388 initialization failed!");
  if (Time - prevTime > 3000)
  {
    if (ledonoff == 1)
    {
      //Turn off
      digitalWrite(led, LOW);
      ledonoff = 0;
    }

    else if (ledonoff == 0)
    {
      //Turn on
      digitalWrite(led, HIGH);
      ledonoff = 1;
    }
    prevTime = Time;
  }
    delay(50);  
  }
  Serial5.println("BMP Success");
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  delay(1000);

/*
  Wire.begin();

  GPS.setI2COutput(COM_TYPE_UBX);
  GPS.setNavigationFrequency(5);

   if (GPS.setDynamicModel(DYN_MODEL_AIRBORNE4g) == false){
    Serial51.println(F("*** Warning: setDynamicModel failed ***"));
    while(1){
    //Led Flasing
    delay(15);
    }
  } 

  GPS.saveConfiguration(); */


    //GPS
   /* if (GPS.begin()) {
        Serial5.println("u-blox GNSS module initialized successfully!");
    } else {
        Serial5.println("u-blox GNSS module initialization failed!");
        while (1);  // If GNSS doesn't initialize, stop the program
    } */


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
  seaLevel = bmp.pressure / 100.0;

  pinMode(led, OUTPUT);
  Serial5.print("Team_ID, ");
  Serial5.print("Mission Time,");
  Serial5.print("Packet Count, ");
  Serial5.print("X Accleration, ");
  Serial5.print("Y Accleration, ");
  Serial5.print("Z Accleration, ");
  Serial5.print("Eular X, ");
  Serial5.print("Eular Y, ");
  Serial5.print("Eular Z, ");
  Serial5.print("Gyro X, ");
  Serial5.print("Gyro Y, ");
  Serial5.print("Gyro Z, ");
  Serial5.print("Pressure ALT, ");
  Serial5.print("Temperature, ");
  Serial5.print("Software State ");
  Serial5.print("\n");
}



void loop() {
  Serial5.print(millis());
  Serial5.print(",");
  getDataFromSensors();
  writeDataToSD();

  startTime = millis();

  if (Time - prevTime > 500)
  {
    if (ledonoff == 1)
    {
      //Turn off
      digitalWrite(led, LOW);
      ledonoff = 0;
    }

    else if (ledonoff == 0)
    {
      //Turn on
      digitalWrite(led, HIGH);
      ledonoff = 1;
    }
    prevTime = Time;
  }

  currentState = LAUNCH_READY;
  switch (currentState) {
  //launchReady(), ascend(), stabilization(), descent(), and landing() are functions that are declared after 
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
    
    
    //delay(50);
  /*if(millis() - startTime < 50) {
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
  */
  
}
}
