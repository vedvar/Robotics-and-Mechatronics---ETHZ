//Group B4: David Goegl, Ved Varshney, Emre Eryilmaz
/// Lab 02 - Analog Signal Acquisition
// Sample the analog output of the hall sensor sensor
// and print the value to the serial port
// use the prewritten functions to move the magnet from one step to the next

// initializiation of motor parameters - DO NOT CHANGE -
char incoming = 0;	  
float position_x = -1.0;   // magnet position in mm
float pos_offset = 16; // distance between hall sensor and face of magnet nearest to it (in mm) when button is pressed.
bool forward = 1;         // definitions of motor direction. Change if motor cable flipped
bool backward = 0;

// Button params
#define switch_pin A3  // Includes 30k pull-up resistor
const int out_pin = 17; // may not be necessary, just connect to logical Vcc
bool switch_press = false;
unsigned int debounceTime = 100; //Time to avoid button debouncing
unsigned long last_pressed = 0;
#define hall_pin A2

// Motor params
#define dirPin A1
#define stepPin 4
#define sleep_pin A0
const int16_t stepsPerRevolution = 200;
const int16_t pitch = 2;  //Pitch in mm
int dir = forward;  //ccw if 1, cw is calibrated
bool reading = false;
bool read_pos = true;

//MY CODE

int sensorvalue;
bool calibrated = false; //initialize uncalibrated


void setup() {
  //Initialize the Input/output pins here
  pinMode(A2, INPUT);
   //pinMode(sleep_pin, OUTPUT);
   
  Serial.begin(115200); // setup the serial connection
  
  // Motor initialization
  attachInterrupt(digitalPinToInterrupt(switch_pin), button_ISR, FALLING);  //when pressed, triggered
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  switch_press = false;
}



void loop() {
  
  digitalWrite(sleep_pin, HIGH); //make motor sleep when not moving


  bool pRead = digitalRead(switch_pin);
  if(pRead ==1) {
     switch_press = false;
     } else {
     switch_press = true;
    }

  // Check if the serial port is available and if a specific character is received from the serial port
  if(Serial.available())
  {
    Serial.readBytes(((char*)&incoming),1);
    
    //return a single readout
    if (incoming == 'a')
    {
      Serial.print(analogRead(A2)); 
    }

    //return average readout from 20 consecutive readouts
    //changed 20 to 50 for Q3 task 5
    else if (incoming == 'b')
    {
      for(int i = 0; i < 50; i++)
      {
        sensorvalue += analogRead(A2);  
      }
      Serial.print(sensorvalue / 50);
      sensorvalue = 0;   // reset the sensor value for the next set of samples
    }

    //return average readout from 200 consecutive readouts
    else if (incoming == 'c')
    {
      for(int i = 0; i < 200; i++)
      {
        sensorvalue += analogRead(A2); 
      }
      Serial.print(sensorvalue / 200);
      sensorvalue = 0;   // reset the sensor value for the next set of samples
    }  
    

//Use the following input characters to move the magnet. Feel free to change the movement.

      else if (incoming == 'w') // move 2mm
      {
        move_motor(2);
      }
      else if (incoming == 'x') // move 10mm
      {
        move_motor(10);
      }
      else if (incoming == 'y') // move 20mm moved to 27mm
      {
        move_motor(27);
      }
      else if (incoming == 'z') //switch direction
      {
        switch_motor_direction();
      }
      
  }
 delay(20);

}
