
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float derivative;
  float reference;
  lasterror = 0;
  lastrefference = reference;


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  // Proportional term
  blank
  error = reference - blank
  //Integral and Derivative terms 
  integral = integral + error * (50 - (millis() - startTime));
  derivative = (error - lasterror) / (50 - (millis() - startTime));
  //Output
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  lasterror = error;

  //The following is psuedocode, we need electrical to tell us where the solenoid mosfets are connected because we can't read schemetics
  //Pins 14 and 15 (A0 and A1) are connected to the solenoids

  if(output >= setvalue) {
    both open solenoids
  }
  else {
    if(output >= setvalue) {
      alternate solenoids
    }
    else {
      both closed solenoids
    }
  }  
}
