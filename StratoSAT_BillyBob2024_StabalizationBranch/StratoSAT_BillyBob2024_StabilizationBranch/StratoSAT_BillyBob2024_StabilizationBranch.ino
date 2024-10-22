
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
  angularvelocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  orienation = bno.getVector(Adafruit_BNO055::VECTOR_EULER); 
  if(angularvelocity >= 10) {
    digitalWrite(solenoidclock, HIGH);
    digitalWrite(solenoidcounter, LOW);
  }
  if(angularvelocity <= -10) {
    digitalWrite(solenoidclock, LOW);
    digitalWrite(solenoidcounter, HIGH);
  }
  error = reference - orientation;
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
}
