#include <stdio.h>
#include "util.h"

int main(){
  //////////////////////////////
  /////// Initialization ///////
  //////////////////////////////

  //Print Welcome Message
  printf("\e[1;1H\e[2J"); // Clear screen
  printf("#####################\n");
  printf("Ball and Plate System\n");
  printf("#####################\n");
  printf("\n");
  printf("Opening serial port...\n");

  // Initialize the serial port
    const char* port= "/dev/ttyUSB0";//vm: "/dev/ttyUSB0", mac: "/dev/cu.SLAB_USBtoUART"
  int fd = serialport_init(port, 115200);
  if (fd == -1){
      printf("Could not open the port.\n");
      printf(" - Is the Arduino IDE terminal opened?\n");
      printf(" - Is the device connected to the VM?\n");
      return -1;    
  }

  // Initialize robot and check
  // if messages are received
  initBallBalancingRobot(fd);

  // Make sure that serial port is relaxed
  usleep(20*1000);

  // Parameter loading functions
  load_parameters();
  load_servo();

  //////////////////////////////
  //////// Task Selection //////
  //////////////////////////////
  int task_selection = 0;
  printf("Select Task: ");
  scanf("%d", &task_selection);

  //////////////////////////////
  /////////// Task 1 ///////////
  //////////////////////////////

  while(task_selection == 1){
    /* Test inverse kinematics via
    terminal */
      
    //initalize variables:
    double plate_angles[] = {0,0};
    double servo_angles[] = {0,0,0};
     
    /* ********************* */
    //USER INPUT 
   printf("Input the pitch and yaw angles: ");
   scanf("%lf", &plate_angles[0]);    //KEYWORD lf for double
   scanf("%lf", &plate_angles[1]);

   //COMPUTE INVERSE KINEMATICS
     int function_check = inverseKinematics(plate_angles, servo_angles);


   // If successful print out the alpha angles for each servo in degrees, rounded to the 1st decimal place
    if (function_check == 0){
     printf("\nAlpha_A = %.1f°,  Alpha_B = %.1f°, Alpha_C = %.1f°\n",servo_angles[0], servo_angles[1], servo_angles[2]);
     }
   
    // If unsuccessful print error message that at least one plate angle is invalid
    if (function_check == -1){
     printf("Plate Angle Not Valid");
     }


    //SEND SERVO-ANGLES TO FEATHERBOARD
     int servo_command_check = servoCommand(fd, servo_angles);

     if(servo_command_check == -1){
       printf("ERROR: Something didn't work.\n");
     }

    // PRINT VALUES and also P_z for the video
    printf("Plate angles: %lf, %lf\n", plate_angles[0], plate_angles[1]);
    printf("Servo angles: %lf, %lf, %lf %f\n", servo_angles[0], servo_angles[1], servo_angles[2], bbs.plate_height);


    //NEW QUERY
      printf("AYO BOY type in 1 if you want to continue plugging in angles:\n");
     
      scanf("%d", &task_selection);
      
    /* ********************* */

  }

  //////////////////////////////
  /////////// Task 2 ///////////
  //////////////////////////////
  /*Test projection from the image frame to the world frame*/
  if(task_selection == 2){
      
      //initalize variables:
      
    /* ********************* */
    int flag;
    int x_in;
    int y_in;
    double x_out;
    double y_out;


   
    while(1){

      if(readFromPixy(fd, &flag, &x_in, &y_in) && flag ){
        //printf("X_in = %d Y_in = %d \n", x_in, y_in);
        project2worldFrame(x_in, y_in, &x_out, &y_out);
        printf(" X_out = %f  Y_out = %f \n \n", x_out, y_out);
 
      }else{
       printf("Blöd gloffa\n");
       
      }
      usleep(50000);
    }
    
   
    /* ********************* */   
  }


  return 0;
}
