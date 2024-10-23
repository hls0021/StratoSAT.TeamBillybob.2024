

void setup() {

}


void loop() {

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

