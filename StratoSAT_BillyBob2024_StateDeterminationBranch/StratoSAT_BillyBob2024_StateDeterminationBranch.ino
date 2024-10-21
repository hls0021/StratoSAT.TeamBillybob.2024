#include <wire.h>
#include <Adafruit_BNO055.h>

//led
int led = 16

//state definition
enum State {
  LAUNCH_READY,
  ASCEND,
  STABILIZATION,
  DESCENT,
  LANDING
};

void setup() {
  tenloops = 0;
  preTime = 900000;
  Serial.begin(9600); 
  pinMode(led, OUTPUT);
}

void loop() {
  //state transition
  switch (currentState) {

    tenloops = tenloops + 1;
    if (tenloops >= 10 & digitalWrite(led, LOW)) {
      digitalWrite(led, HIGH);
      tenloops = tenloops - 10;
    }
    else {
      if(tenloops >= & digitalWrite(led, HIGH)) {
        digitalWrite(led, LOW);
        tenloops = tenloops - 10;
      }
    }

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
  if (altitude > 1.0) {
    currentState = ASCEND;
  }
}

//ascend state
void ascend() {
  if (altitude > 16.0) {
    currentState = STABILIZATION;
  }
}

//stabilization state
void stabilization() {
  if (threshold > value) {
    currentState = DESCENT;
  }
}

//descent stae
void descent() {
  if (pressure <= upper & presure >= lower) {
    if (velocity = 0) {
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

