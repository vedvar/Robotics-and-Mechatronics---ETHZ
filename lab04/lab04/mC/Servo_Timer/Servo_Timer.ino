/**
 * If we want to move by 180 degrees in 150 seconds we have to move 1.2 degrees per second
 * 
 * If 550 us correcponds to o degrees and 2400us corresponds to 180 degrees, with a linear interpolation i get:
 * 562.33333333 periodically
 */


// Make sure these libraries are installed
// If not the case, use Tools > Manage Libraries
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// declaring variables

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int servo_num = 0;
int servo_freq = 50;

int pos, MinPulse, MaxPulse;  // current position, minimum (0°) & maximum (180°) pulse length of the servo in µs             
float secondStep = 12.33333333;             // unrounded pulse length increase in µs that corresponds to an increase of 1 second
float exactPos = 2400;               // Exact, unrounded position of the servo in µs
char serialVariable;          // Character received through the serial communication
const int USMIN = 550;
const int USMAX = 2400;

void setup() {
  Serial.begin(115200);                 // open serial communication
  Serial.println("Lab 04 - Emre, Ved, David");

  //_________________Begin - Setup_______________
  pwm.begin();
  
  pwm.setPWMFreq(50);  
  pwm.writeMicroseconds(0, USMAX);
  delay(10);
 
  //_________________End - Setup_______________

}

int floatToInt(float val)
{
  return val;
}

void loop() {
  
  // Only run the loop if the serial communication is available
  if( Serial.available())
  {
    
    // read the incoming character and save it in "serialVariable"
    serialVariable = Serial.read();

    // If "a" is received reset the servo to its starting position.
    
    //_________________Begin - Reset_______________

    if(serialVariable == 'a'){
        pwm.writeMicroseconds(0, USMAX);
        exactPos = 2400;
    }
 
    //_________________End - Reset_______________


    // If "b" is received move the motor by one step.
    
    //_________________Begin - Advance_______________
      if(serialVariable == 'b'){
        exactPos = exactPos -secondStep;
        int intValue = floatToInt(exactPos);
       pwm.writeMicroseconds(0, intValue);
    }
 
    //_________________End - Advance_______________

    
  }
}
