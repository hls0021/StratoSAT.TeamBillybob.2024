#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

float tenloops;
float preTime;
float endTime;
float altitude;
float gyro;
float ledonoff = 0;


//led
int led = 16;

//state definition
enum currentState {
  LAUNCH_READY,
  ASCEND,
  STABILIZATION,
  DESCENT,
  LANDING
};

void setup() {
  Serial.begin(9600); 
  pinMode(led, OUTPUT);
}

void loop() {
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

    case ASCEND:
    ascend();

    case STABILIZATION:
    stabilization();

    case DESCENT:
    descent();

    case LANDING:
    landing();

  }
}

//launch ready state
void launchready() {
  if (altitude > 100000) {
    currentState = ASCEND;
  }
}

//ascend state
void ascend() {
  if (altitude > 1600000 {
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
  break
}

