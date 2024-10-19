
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float derivative;
  float integrallimit;
  float reference;
  lasterror = 0;
  lastrefference = reference;


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  //Integral and Derivative terms 
  integral = integral + error * 50;
  derivative = (error - lasterror) / 50;
  if ()
  error = lasterror;


}
