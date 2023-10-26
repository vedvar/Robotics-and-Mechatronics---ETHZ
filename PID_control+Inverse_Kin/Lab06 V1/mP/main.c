#include <stdio.h>
#include "util.h"

int main()
{
  //////////////////////////////
  /////// Initialization ///////
  //////////////////////////////

  // Time information
  time_t rawtime;
  struct tm *info;
  char buffer[80];
  time(&rawtime);
  info = localtime(&rawtime);
  printf("Formatted date & time : |%s|\n", buffer);

  // Print Welcome Message
  printf("\e[1;1H\e[2J"); // Clear screen
  printf("#####################\n");
  printf("Ball and Plate System\n");
  printf("#####################\n");
  printf("|%s|\n", buffer);
  printf("\n");
  printf("Opening serial port...\n");

  // Initialize the serial port
  const char *port = "/dev/ttyUSB0";
  int fd = serialport_init(port, 115200);
  if (fd == -1)
  {
    printf("Could not open the port.\n");
    printf(" - Is the Arduino IDE terminal opened?\n");
    printf(" - Is the device connected to the VM?\n");
    return -1;
  }

  // Initialize robot and check
  // if messages are received
  initBallBalancingRobot(fd);

  // Make sure that serial port is relaxed
  usleep(20 * 1000);

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

  if (task_selection == 1) //lab05 used a while loop instead of if
  {
    /* Test inverse kinematics via
     terminal */

    // initalize variables:
    double plate_angles[] = {0, 0};
    double servo_angles[] = {0, 0, 0};

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
  //x///////// Task 2 ///////////
  //////////////////////////////
  /*Test camera calibration*/
  if (task_selection == 2)
  {
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
        cameraCalibration(x_in, y_in, &x_out, &y_out);  //changed project2worldframe to cameraCalibration
        printf(" X_out = %f  Y_out = %f \n \n", x_out, y_out);
 
      }else{
       printf("Blöd gloffa\n");
       
      }
      usleep(50000);
    } 
  }


  //////////////////////////////
  /////// Task 4/5/6 ///////////
  //////////////////////////////

  if ((task_selection == 4) || (task_selection == 5) || (task_selection == 6))
  {

    // TODO: Initialize default PID parameters
    double k_p = 0.03;
    double k_d = 0.00;
    double k_i = 0.00;

    // TODO: Intialize filter window size
    int n_pos = 10;
    int n_vel = 10;
    double declare = 0;

    /*
    // TODO: Ask for user input to change PID parameters
    printf("Type in 0 to use default parameters or 1 to use your own parameters");
    scanf("%lf \n", &declare);
    if(declare == 1){
      printf("Type in the parameters ofr k_p, k_d, k_i");
      scanf("%lf", &k_p);    //KEYWORD lf for double
      scanf("%lf", &k_d);
      scanf("%lf", &k_i);
    }
    */

    // Variables for Pixy2
    int flag = 0;      // flag that detects if the pixy cam can detect a ball
    int x_px = 0;      // raw x coordinate read by pixy cam
    int y_px = 0;      // raw y coordinate read by pixy cam
    double x_cal = 0;  // calibrated x coordinate in plate frame and mm
    double y_cal = 0;  // calibrated y coordinate in plate frame and mm
    double x_filt = 0; // filtered x coordinate in plate frame and mm
    double y_filt = 0; // filtered y coordinate in plate frame and mm
    double vel_x = 0;  // x velocity calculated from filtered position
    double vel_y = 0;  // y velocity calculated from filtered position

    //FABIO used those
    double vel_x_filt = 0;  // x velocity filtered
    double vel_y_filt = 0;  // y velocity filtered

    // read pixy a couple of times to clear buffer
    for (int i = 0; i < 20; i++)
    {
      readFromPixy(fd, &flag, &x_px, &y_px);
    }

    // create buffer arrays for filtered variables
    // [0] is always the current element
    // make sure buf_size is bigger than filter windows
    int buf_size = 50;
    double x_raw[buf_size]; // calibrated, unfiltered
    double y_raw[buf_size];
    double vx_raw[buf_size]; // 1st order derivative
    double vy_raw[buf_size];
    double x[buf_size]; // filtered position
    double y[buf_size];
    double vx[buf_size]; // filtered velocity
    double vy[buf_size];

    // initialize buffer arrays to zero
    for (int i = 0; i < buf_size; i++)
    {
      x_raw[i] = 0;
      y_raw[i] = 0;
      vx_raw[i] = 0;
      vy_raw[i] = 0;
      x[i] = 0;
      y[i] = 0;
      vx[i] = 0;
      vy[i] = 0;
    }

    // initialize angles
    double servo_angles[] = {0, 0, 0};
    double plate_angles[] = {0, 0};

    // reference variables for control
    // are being set in reference functions
    double x_ref, y_ref, vx_ref, vy_ref;

    /*
    // pid variables
    double x_integ = 0;
    double y_integ = 0;
    double u_x = 0;
    double u_y = 0;
    */

    // define error variables
    double  e_x, e_y, e_vx, e_vy;
    double prev_e_x = 0;
    double prev_e_y = 0;

    // pid variables
    double x_prop = 0;
    double y_prop = 0;
    double x_integ = 0;
    double y_integ = 0;
    double x_derv = 0;
    double y_derv = 0;
    double u_x = 0;
    double u_y = 0;

    // Logfile with datetime as filename
    char datetime[80];
    strftime(datetime, 80, "%Y-%m-%d_%H-%M-%S_pid_log.txt", info);
    FILE *fp = fopen(datetime, "w+");
    startLogging(fp, task_selection, k_p, k_d, k_i, n_pos, n_vel);

    // Timing variables
    //  TODO: Measure the current sampling time dt (in seconds, for the derivative): It is the time it takes to run the previous loop iteration.
    //  Hint: use getMicroseconds() and don't forget to convert to seconds.
    long start = 0;
    long end = 16;
    long t0 = getMicroseconds(); // get starting time of the loop
    double dt = 0.016;           // variable for timing
    double current_time = 0;

    while (1)
    {

      /* ********************* */
      /* Insert your Code here */
      /* ********************* */

      // TODO: Get current sampling time dt
      dt = (end - start)/1000000.0; //dt must be seconds, "end" and "start" given in microseconds
      start = getMicroseconds();

      // TODO: Get the coordinates of the ball in the Pixy Camera frame (Use a function in util.c)
      readFromPixy(fd,&flag,&x_px,&y_px);

      // If the ball is detected, enter if-bracket
      if (flag)
      {
        // TODO: Use camera calibration form Lab05
        cameraCalibration(x_px, y_px, &x_cal, &y_cal);

        // TODO: Place measurements in buffer array
        // Hint: There is a function called pushBack
        //  in util.h that you can use here.
        pushBack(x_cal,x_raw,buf_size);
        pushBack(y_cal,y_raw,buf_size);

        // TODO: Apply filter to position coordinates
        x_filt = movingAverage(n_pos, x_raw);
        y_filt = movingAverage(n_pos, y_raw);
        pushBack(x_filt,x,buf_size);
        pushBack(y_filt,y,buf_size);

        // TODO: Compute velocity based on the filtered position signal
        vel_x = discreteDerivative(dt, x);
        vel_y = discreteDerivative(dt, y);

        // TODO: Place velocity in buffer array (use pushBack function)
        pushBack(vel_x,vx_raw,buf_size);
        pushBack(vel_y,vy_raw,buf_size);


        
        // TODO: Apply filter to velocity
        vel_x_filt = movingAverage(n_vel, vx_raw);
        vel_y_filt = movingAverage(n_vel, vy_raw);
        pushBack(vel_x_filt,vx,buf_size);
        pushBack(vel_y_filt,vy,buf_size);


        // TODO: Set reference depending on task 
        switch (task_selection)
        {
        case 4: //TODO: Postlab Q4 centering task 
          x_ref = 0;
          y_ref = 0;
          vx_ref = 0;
          vy_ref = 0;
          break;

        case 5: //TODO: Postlab Q5 step response reference  --> use function in util.h 

          stepResponse(getMicroseconds()/1000000.0, &x_ref, &y_ref, &vx_ref, &vy_ref);

          break;

        case 6: //TODO: Postlab Q6 circular trajectory reference --> implement & use function in util.h 

          circularTrajectory(getMicroseconds()/1000000.0, &x_ref, &y_ref, &vx_ref, &vy_ref);

          break;
        }

        // Calculate the current errors -> FABIO did that
          e_x = x_ref - x_filt;
          e_y = y_ref - y_filt;
          e_vx = vx_ref - vel_x_filt;
          e_vy = vy_ref - vel_y_filt;

        // TODO: Update Integrator after an initial delay
        // Hint: Wait 0.5s before starting to update integrator
          x_integ += dt*((e_x+prev_e_x)/2); //why prev_e_x??
          y_integ += dt*((e_y+prev_e_y)/2);


        // TODO: Compute PID (remember, PID output is the plate angles)
          x_prop = e_x;
          y_prop = e_y;
          x_derv = -e_vx; //prev it was a (-)
          y_derv = -e_vy;

          u_x = k_p*x_prop + k_i*x_integ +k_d*x_derv; //u_pid for x-axis
          u_y = k_p*y_prop + k_i*y_integ +k_d*y_derv; //u_pid for y-axis

          // store current error as previous error
          prev_e_x = e_x;
          prev_e_y = e_y;


        // TODO: Define Plate angles from PID output (watch out for correct sign)
          plate_angles[0] = u_y;
          plate_angles[1] = -u_x;

        // TODO: Compute servo angles and send command
        inverseKinematics(plate_angles, servo_angles);

        servoCommand(fd,servo_angles);



        // Open logging file and log everything to textfile
        fp = fopen(datetime, "a");
        logger(fp, end, current_time, dt, k_p, k_d, k_i, x_ref, y_ref, vx_ref,
               vy_ref, x_raw[0], y_raw[0], x[0], y[0], vx_raw[0], vy_raw[0],
               vx[0], vy[0], plate_angles, servo_angles, x_integ, y_integ);
      }

      end = getMicroseconds();
    }
  }

  return 0;
}
