#include <PinChangeInt.h>
#include <PID_v1.h>                                   // Thanks to Brett Beauregard for his nice PID library http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
#define encodPinA1      2                             // Quadrature encoder A pin
#define encodPinB1      8                             // Quadrature encoder B pin
#define M1              9                             // PWM outputs to L298N H bridge motor driver module
#define M2              10
double kp = 5 , ki = 1 , kd = 0.01 ,input = 0, output = 0, setpoint = 0;   // modify kp, ki and kd for optimal performance
long temp;
volatile long encoderPos = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'

void setup() {
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(0, encoder, FALLING);               // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31Kh PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
}
void loop() {
  temp += analogRead(0);                              // increment position target with potentiometer value (speed)
  if (temp < 0) {                                     // in case of overflow
    encoderPos = 0;
    temp = 0;
  }
  setpoint = temp / 500;                              // modify division to fit motor and encoder characteristics
  input = encoderPos ;                                // data from encoder
  Serial.println(temp);
  myPID.Compute();                                    // calculate new output
  pwmOut(output);                                     // drive L298N H bridge module
}
void pwmOut(int out) {                                // to H-Bridge board
  if (out > 0) {
    analogWrite(M1, out);                             // drive motor CW        
    analogWrite(M2, 0);                                              
  }
  else {
    analogWrite(M1, 0);                         
    analogWrite(M2, abs(out));                        // drive motor CCW
  }
}
void encoder()  {                                     // pulse and direction, direct port reading to save cycles
  if (PINB & 0b00000001)    encoderPos++;             // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      encoderPos--;             // if (digitalRead(encodPinB1)==LOW)   count --;
}





