//David Gögl, Ved Varshney, Emre Eryilmaz
// declaring variables
int dl, mlt;
int  incoming = 0;

// define an array to access the LED Pins [These are the IO pis we want to access]
int led_pins[10] = {12, 27, 33, 15, 32, 14, 22, 23};
int switch_pin = 21; //define the pin of where the switch is connected

void setup() {
  // setting some initial values
  dl = 10;   //The first four bits are stored here
  mlt = 50;  //The last four bits are stored here

  //_________________Begin - Part A_______________


  //Begin Serial communication
  Serial.begin(115200);

  //Set pin 21 to input
  pinMode(switch_pin, INPUT);

  //Set the pin modes of the 8 pins to OUTPUT
  int j = 0;
  for (j = 0; j < 8; j++) {
    pinMode(led_pins[j], OUTPUT);
  }

  //_________________End - Part A_________________
}

void loop() {
  int i = 0, j = 0;
  //_________________Begin - Part B_______________

  if ( Serial.available() )
  {
    // read into variable incoming by passing a memory address where the value of incoming is stored.
    // the function will  read one byte (i.e. 8 bits) of data from the serial input and write it to the memory address pointed to by (char*)&incoming.
    Serial.readBytes(((char*)&incoming), 1);
    Serial.println(incoming);    //Prints out the corresponding ASCII number


    dl = incoming >> 4;        //Divide the incoming value by 16 ( Same as shifting by 4 places)
    mlt = incoming % (1 << 4); //Calculate the remainder of incoming / 16

  }

  //_________________End - Part B_________________


  //_________________Begin - Part C_______________

  // loop over all output (LED) pins and set state to HIGH/LOW
  // e.g. digitalWrite(1,LOW) means you set the digital pin 1 to LOW.
  if (digitalRead(switch_pin) == 0) {
    for (j = 0; j < 8; j++) {
      for (i = 0; i < 8; i++)
        //Turn of ALL 8 pins
        digitalWrite(led_pins[i], LOW);
      //Only turn on the one pin at position j
      digitalWrite(led_pins[j], HIGH);
      delay(dl * mlt);
    }
    delay(dl * mlt * 3);
  }

  //_________________End - Part C_________________


  //_________________Begin - Part D_______________

  //Convert dl and mlt into two arrays that represent the binary code of those numbers
  if (digitalRead(switch_pin) == 1) {
    int incoming_b[8];  //This is the array we will save the bits of our incominc DEC number in (Format: BIN)

    //Filling Array with the bits (Basically DEC -> BIN conversion)
    for (i = 0; i < 8; i++) {
      incoming_b[i] = (incoming >> i) & 1;
    }
    //Light the correct LED's
    for (j = j; j < 8; j++) {
      digitalWrite(led_pins[j], incoming_b[j]);
    }
  }

  //_________________End - Part D_________________

}
