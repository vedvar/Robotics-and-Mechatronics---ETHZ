//----------------------------------------------------------------------------------------------------------------------------------
//APPENDIX THEORY

//Variables 
Type      Declaration                  # of Bits Range
intN_t    N-bit, N ∈ {8,16,32,64}     [-2^(N-1), +2^(N-1)-1]
uintN_t   N-bit, N ∈ {8,16,32,64}     [0, 2^N-1]

1 Byte = 8 Bits

//Pointers
int* ptr;   //Initialise a pointer with name ptr.
ptr = &k;   //Save the adress of k in ptr.
*ptr = 7;   //Change the value of the variable where ptr points to to 7.
printf("%d\n",*ptr);  //Print the value of the variable where ptr points to.
struct tag *st_ptr = &my_struct;  //Pointer to a structure
(*st_ptr).age = 63;     //Acces a object in a structure
st_ptr->age = 63;   //The BETTER version of the above

//Structs

#include <stdio.h>
#include <string.h>

struct tag {
char lname[20]; /* last name */
char fname[20]; /* first name */
int age; /* age */
float rate; /* e.g. 12.75 per hour */
};

struct tag my_struct; /* declare the structure my_struct */

int main(void)
{ 
strcpy(my_struct.lname,"Jensen");  //assigns the value "Jensen" to the lname member variable of the my_struct struct.
strcpy(my_struct.fname,"Ted");
printf("\n%s ",my_struct.fname);
printf("%s\n",my_struct.lname);
return 0;
}


//----------------------------------------------------------------------------------------------------------------------------------







//LINUX

//In file (rechts)
cd name
//In file (links)
cd ..
//Alles auflisten
ls
//Alles aufräumen
clear
//Wo bin ich ? 
pwd
//Copy file
cp 
//Neues file erstellen
mkdir
//file löschen
rmdir
//execute a program
./programName
//Open something in SublimeText
subl fileName
//Alles compilen
make 
//Alles löschen ???
make clean

//Compile code-------------------------
(1) CTRL + Shift + T
(2) Are you in folder containing that code?
(3) irm@ubuntu:~/Desktop/irm/lab00 $ gcc hello_world.c
(4) irm@ubuntu:~/Desktop/irm/lab00 $ ./a.out
(5) Rename the a.out file: irm@ubuntu:~/Desktop/irm/lab00 $ gcc -o hello_world hello_world.c
//------------------------------------


//Neues Programm in SublimeText erstellen
irm@ubuntu:~/Desktop/irm/lab00 $ subl nameOfProgram.c


//----------------------------------------------------------------------------------------------------------------------------------







//----------------------------------------------------------------------------------------------------------------------------------

//C
//Grundstruktur
#include <stdio.h>
#include <stdint.h>
#include "fileName.h"
int main()
{
//CODE
return 0;
}

//Array
double arrayName[] = {0,0};

//Read Input
int num1;
scanf("%d", &num1);                   
printf("Text: %d \n", num1);   //%f, %c, %p, %x

//Cast the value 245 to a 8-bit unsigned integer.
  uint8_t arg = (uint8_t)245;

//SERIAL COMMUNICATION
int fd = serialport_init( "/dev/ttyUSB0", 115200);
int n = serialport_writebyte(fd, ((char*)&arg));    //This line of code sends a byte of data stored in the memory location pointed to by the address of variable 'arg' over the serial port using the file descriptor 'fd
  if (n == -1 ) printf("Unable to write the port \n");
int n = serialport_write(fd, "a"); 
        int w = serialport_read(fd, charBuffer, 4, 1000); // Read the sensor value from the serial port into the buffer 
        value = atoi(charBuffer);  //converting charBuffer to int with atoi
serialport_close(fd);

//Nice C-Codes:
  //Print Bits with zero-padding and spaces every 4 bits
void print_bits(uint8_t arg_word){
  int count = 0; //used to space the binary number after counting up to 3
  int binarray[8];
  for (int i = 0; arg_word > 0; i++){
    binarray[i] = arg_word % 2;
    arg_word = arg_word / 2; }
  //zero padding: Fill the array with zeros after the most significant 1
  for (int k = 0; k < 8; k++){
    if (binarray[k] != 1) {
      binarray[k] = 0;}}
  //print out the array and make a space after 4 bits
  for (int j = 7; j >= 0; j--){
    printf( "%d", binarray[j]);
    count++;
    if (count == 4) {
      printf(" ");
      count = 0;}}
  printf("\n");}

//Convert the measured voltage value to a magnetic field
float hall_sensor_get_field(float voltage, float voltage_0)
{return (voltage - voltage_0) * (1 / 0.03);}

//COMPUTE INVERSE KINEMATICS
     int function_check = inverseKinematics(plate_angles, servo_angles);
//SEND SERVO-ANGLES TO FEATHERBOARD
     int servo_command_check = servoCommand(fd, servo_angles);
//PROJECTION
    if(readFromPixy(fd, &flag, &x_in, &y_in) && flag ){
        project2worldFrame(x_in, y_in, &x_out, &y_out);}
//NEWTON-RAPHSON
    //Function
      double func(double r, double k1, double k2, double r_d){
        return r + k1*pow(r, 3) + k2*pow(r, 5) - r_d; }
    // Derivative of the above function f(r)
    double derivFunc(double r, double k1, double k2){
        return 1 + 3*k1*pow(r, 2) + 5*k2*pow(r, 4);}
    //NewtonRaphson
    double newtonRaphson(double r_d, double k1, double k2){
       double r = r_d; // initial guess
        double h = func(r,k1,k2,r_d) / derivFunc(r,k1,k2);
        int nr_iterations = 0;
        while (fabs(h) >= EPSILON && nr_iterations < 1000){
            h = func(r,k1,k2,r_d)/derivFunc(r,k1,k2);
            r = r - h;
            nr_iterations = nr_iterations +1;
        }   return r; }   

//PROJECT TO WORLD FRAME

int project2worldFrame(const int x_in, const int y_in, double* x_out, double* y_out){
//Scale factor
  int x = bbs.calibration_image_scale * x_in;
  int y = bbs.calibration_image_scale * y_in;
//Normalize shifted Input
  double x_norm = (x - bbs.distortion_center[0])/bbs.focal_length;
  double y_norm = (y - bbs.distortion_center[1])/bbs.focal_length;
  //Newton-Raphson
  double r_d = sqrt(pow(x_norm,2.0) + pow(y_norm,2.0));
  double r = newtonRaphson(r_d,  bbs.radial_distortion_coeff[0], bbs.radial_distortion_coeff[1]);
  //printf("r = %f \n", r);
  double denom = 1 + bbs.radial_distortion_coeff[0] * pow(r,2) + bbs.radial_distortion_coeff[1] * pow(r,4) ;
  double u = (1 / denom) * x_norm;
  double v = (1 / denom) * y_norm;
  //the total height needed for calculations
  double z_c = bbs.plate_height + bbs.ball_radius + bbs.cam_offset[2];
  //Calculate the output in world frame
  *x_out = u * z_c - bbs.cam_offset[0];
  *y_out = v * z_c - bbs.cam_offset[1];
  return 0;
};


















//PYTHON

//#General
  //#Shit to import
  %matplotlib inline
  import numpy as np
  import os
  from matplotlib import pyplot as plt
  from scipy.optimize import curve_fit
  from scipy import signal
  import control
  from control import TransferFunction, tf, bode_plot, step_response, impulse_response

  //#For loops
  for i in range(10):
      print("The number: " + str(number))

  //#if loops
    if x > 5:
      print("x is greater than 5")
  else:
      print("x is less than or equal to 5")

  //#Functions
      def add_numbers(x, y):
      return x + y

  result = add_numbers(5, 10)

  //#Define frequency range ( 50001 equally spaced numbers between 0 and 5, inclusive )
  T = np.linspace(0, 5, 50001)

//#HANDLE DATA FROM TXT-FILES
  //#Import
  data_dir = r"lab02/mP/lab02_skeleton.ipynb"
  voltage_data_files = ['voltage_data_a.txt', 'voltage_data_b.txt', 'voltage_data_c.txt']
  for file_num, file in enumerate(voltage_data_files):
    path = os.path.join(data_dir, file)
    v_data = np.loadtxt(voltage_data_files[file_num])
    

  //#Make Plots
    //Simple Plot
    plt.rcParams["figure.figsize"] = [14, 3.50]
    plt.rcParams["figure.autolayout"] = True
    plt.title("Ratio of F to I")
    data1 = np.array([1, 2, 3]) 
    data2 = np.array([1, 2, 3] )
    plt.plot(data1, data2)

    //Subplots
    fig, axs = plt.subplots(1, 3, sharey=True, figsize=(15,5))   
    fig.suptitle('BIG TITLE', fontsize=20)
    axs[file_num].set_title(file)
    axs[0].set_ylabel('Voltage')
    axs[2].set_xlabel('Average 200 readouts')
    axs[file_num].plot(v_data)
    plt.savefig('noise_analysis.png', dpi=300)
    plt.show()

//#TRANSFER FUNCTIONS 
  //# Define transfer function of plant
  tau = 1/(2*np.pi*fc)
  num = [3*10]     #The order is s², s¹, s, 1
  den = [5, tau, 0]  #The order is s², s¹, s, 1
  P = control.tf(num, den)
  print('P(s) = ', P)

  //#Bode-Plot
  mag,phase,omega = control.bode_plot(G, Hz = True, dB = True)

  //#Step-Antwort
  t_step, y_step = control.step_response(P, T=5, X0=0.0)//Hatch the data

  axs[0].plot(t_step, y_step)
  axs[0].set_title('Step Response')
  axs[0].set(xlabel = 'Time (s)', ylabel = 'Output')
  axs[0].plot(t_step, y_step,'tab:blue')

  //#Impuls-Antwort
  t_imp, y_imp = control.impulse_response(P, T=5, X0=0.0, input=0.1) //Wie oben empfohlen







//----------------------------------------------------------------------------------------------------------------------------------
//ARDUINO 

//CODE_LAYOUT
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT);
  //Automatically change a variable name every ccurrence of dirPin in the code will be replaced by A1 during compilation.
  //So, if you have a line of code like pinMode(dirPin, OUTPUT);, it will be equivalent to pinMode(A1, OUTPUT); after preprocessing
   #define dirPin A1
}

// the loop routine runs over and over again forever:
void loop() {
  /*SERIAL-Read ------------------------------*/
  Serial.readBytes(((char*)&incoming),1);  //Reads 1 Byte, CAST to char and save at adress of incoming
  serialVariable = Serial.read();       //// read an incoming character
  Serial.println(sensorValue);
  if(Serial.available()){} //Ok if atleast one byte available
  delay(1);        // delay in between reads for stability
  /*ANALOG-Read ------------------------------*/
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0); 
  /*DIGITAL-Read ------------------------------*/
  int buttonState = digitalRead(pushButton);
  digitalWrite(LED_BUILTIN, HIGH);
  //for loops
  int j = 0;
  for (j =0;j<8;j++){ }
  //Arrays
  int myArray[5] = {1, 2, 3, 4, 5};  //Initialize
}



//Nice ARDUINO Codes:
  //Nibble-Trenner
  links = incoming >> 4; //Divide the incoming value by 16 ( Same as shifting by 4 places)
  rechts = incoming % (1 << 4);

  //DEC -> BIN
    for (i = 0; i < 8; i++) {
      incoming_b[i] = (incoming >> i) & 1;
    }

  //Servo
    // create a pwm object to control the servo
    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
    //in setup
    pwm.begin();
    pwm.setPWMFreq(servo_freq); 

  //Functions
    int square(int x) {
    return x * x;
    } 

    void printMessage(String message) {
    Serial.println(message);
    }



//LABS

//----------------------------------------------------------------------------------------------------------------------------------
//lab02

//-> The analog-to-digital
  converter (ADC) on the Featherboard has a maximum resolution of 12-bits. The voltage measured is then assigned
  to a value between 0 and 4095 (= 212 - 1)
  
  12 BITS:
  -> V = 3.3*(digital reading)/4095 when the voltage range is 0 to 3.3V.
  
  16 BITS:
  -> V = 10.0*(digital reading)/32767 when the voltage range is +/- 10V.
  -> V = 5.0*(digital reading)/32767 when the voltage range is +/- 5V.

//-> Know how to save your variables
  int16_t hex1 = 0xD8F0;  //outputs -10000
  int32_t hex2 = 0xD8F0;  //outputs 55536

  -> In the above case,
    0xD8F0 had a "1" at the MSB, and is therefore a negative number. However, int32_t is a 32-bit integer where
    the leftmost bit is the sign bit. Thus, an int32_t variable stores 0xD8F0 as 0x0000D8F0.

//->Resolution:
The voltage resolution of the ADC is dependent on both the input voltage range and the bit size. For the ADC on
the Featherboard,
Voltage range = 0 to 3.3 -> Resolution = (3.3 - 0)/4095 = 0.806 mV.


//Communication between Featherboard and Computer

->Always via Serialport:
  int fd = serialport_init( "/dev/ttyUSB0", 115200);
  serialport_write(fd, "x");   

  int w = serialport_read(fd, charBuffer, 4, 1000); 
  value = atoi(charBuffer);  

//----------------------------------------------------------------------------------------------------------------------------------
