#include <wire.h>
#include <Adafruit_BNO055.h>

//state definition
enum State {
  LAUNCH_READY,
  ASCEND,
  STABILIZATION,
  DESCENT,
  LANDING,
  END
};

void setup() {

}

void loop() {
  //state transition
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

    case END:
    end;
  }
}