
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float derivative;
  float reference;
  float lasterror = 0;

  sensor_event_t event;
  int solenoidclock = 14;
  int solenoidcounter = 15;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  // Proportional term
  bno.getevent(&event);
  orienation = bno.getVector(Adafruit_BNO055::VECTOR_EULER); 
  error = reference - orientation;
  //Integral and Derivative terms 
  integral = integral + error * (50 - (millis() - startTime));
  derivative = (error - lasterror) / (50 - (millis() - startTime));
  //Output
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  lasterror = error;

  if(output >= setvalue) {
    digitalWrite(solenoidclock, HIGH);
    digitalWrite(solenoidcounter, LOW);
  }
  if(output <= setvalue) {
    digitalWrite(solenoidclock, LOW);
    digitalWrite(solenoidcounter, HIGH);
  }  
  if(output >= negativezero) {
    if(output <= positivezero) {
      digitalWrite(solenoidclock, LOW);
      digitalWrite(solenoidcounter, LOW);
    }
  }
}
