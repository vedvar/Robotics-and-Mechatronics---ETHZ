// Make sure these libraries are installed
// If not the case, use Tools > Manage Libraries
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
int poti = A0;

// declare variables
int servo_num = 0;
int servo_freq = 50;
const int USMIN = 550;
const int USMAX = 2400;

// our servo # counter
uint8_t servonum = 0;

// create a pwm object to control the servo
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  
  // _______________ Begin - Setup _______________

  // Begin the serial communication
  Serial.begin(115200); // setup the serial connection
  
  // Begin PWM communication and set servo frequency
  pwm.begin();
  pinMode(A0, INPUT);
  pwm.setPWMFreq(servo_freq);  // Analog servos run at ~50 Hz updates

  
  // _______________ End - Setup _______________
    delay(10);
}

void loop() {
  // _______________ Begin - Loop _______________

  // Read values from the analog pin and map/scale them to the movment range of the servo.
   int potiValue = analogRead(A0);
   Serial.println(potiValue);
   int mappedValue = map(potiValue, 0, 4095, USMIN, USMAX);   //???
   pwm.writeMicroseconds(servonum, mappedValue);
   delay(50); // wait for servo to move to the desired position

  // Optionally display the reading from the potentiometer on the serial monitor
  // Set the servo position according to the mapped/scaled value
  
  // _______________ End - Loop _______________
  
}
