#include "util.h"
#include "newton_raphson.h"





//_______________________________________________________________________________________________________________________________
int inverseKinematics(double *plate_angles, double *servo_angles)
{
    // Load parameters R, L_1, L_2, P_z etc. from parameters file. Example: double R = bbs.R_plate_joint;
    // Then implement inverse kinematics similar to prelab
  float L1 = bbs.l1;            //Length of Segment 1 in mm
  float L2 = bbs.l2;           //Length of Segment 2 in mm
  float P_z = bbs.plate_height;          //Height of Plate center in mm
  float R = bbs.R_plate_joint;            //Distance from Plate center to anchor points A,B and C in mm
  float beta[3];            //Initialize array to store beta servo angles
  
  
   //Transform plate angles from degrees to radians for calculation
   
   plate_angles[0] = plate_angles[0] * M_PI/180.0;
   plate_angles[1] = plate_angles[1] * M_PI/180.0;
  
  
  // Calculate the z-offsets of each anchor point according to the formulas
  
  float delta_z_A = R * sin(plate_angles[0]); 
  float delta_z_B = - 0.5 * R * sin(plate_angles[0])+ (sqrt(3)/2.0) * R * sin(plate_angles[1]); //index 0 = phi_x; index 1 = theta_y
  float delta_z_C = - 0.5 * R * sin(plate_angles[0])- (sqrt(3)/2.0) * R * sin(plate_angles[1]);

  
  // Calculate the beta-servo angles according to the law of cosines
  
  beta[0] = acos((pow(P_z + delta_z_A, 2) + pow(L1, 2) - pow(L2, 2))/(2 * (P_z + delta_z_A) * L1));     //für beta_A
  beta[1] = acos((pow(P_z + delta_z_B, 2) + pow(L1, 2) - pow(L2, 2))/(2 * (P_z + delta_z_B) * L1));     //für beta_B
  beta[2] = acos((pow(P_z + delta_z_C, 2) + pow(L1, 2) - pow(L2, 2))/(2 * (P_z + delta_z_C) * L1));     //für beta_C
  
   
  // Return -1 if any of the angles is not defined; 
  //beta servo angles should be in range [0 - 180°] because alpha angles are in range [-90°; +90°]
  // isnan-function checks if beta angle is a real number or "nan" not-a-number
  
  if((beta[0]<0) || (beta[0]>M_PI) || isnan(beta[0])) return -1;
  if((beta[1]<0) || (beta[1]>M_PI) || isnan(beta[1])) return -1;
  if((beta[2]<0) || (beta[2]>M_PI) || isnan(beta[2])) return -1;
  
  // Calculate the alpha servo angles and transform them back to degrees
  
  servo_angles[0] = 180.0/M_PI * (M_PI/2.0 - beta[0]); //   pi/2 - beta[] um zu alpha servo angle konvertieren; 180/pi um wieder in rad umzurechnen
  servo_angles[1] = 180.0/M_PI * (M_PI/2.0 - beta[1]);
  servo_angles[2] = 180.0/M_PI * (M_PI/2.0 - beta[2]);


  return 0; // if ok
}


//_______________________________________________________________________________________________________________________________
int cameraCalibration(const int x_in, const int y_in, double *x_out, double *y_out)
{


    // implement the code to project the coordinates in the image frame to the world frame
    // make sure to multiply the raw pixy2 coordinates with the scaling factor (ratio between
    // image fed to python for calibration and pixy2 resolution): bbs.calibration_image_scale.

    double u_norm;
    double v_norm;
    double r_d;
    double u_undist;
    double v_undist;
    double x_w;
    double y_w;
    double z_w;
    double lambda; // lambda = Z_c

    //normalise
    u_norm = (x_in*bbs.calibration_image_scale-bbs.distortion_center[0])/bbs.focal_length;
    v_norm = (y_in*bbs.calibration_image_scale-bbs.distortion_center[1])/bbs.focal_length;

    r_d = sqrt(pow(u_norm,2)+pow(v_norm,2));

    double r = newtonRaphson(r_d, bbs.radial_distortion_coeff[0], bbs.radial_distortion_coeff[1]);

    //undistortion
    u_undist = (1/(1+bbs.radial_distortion_coeff[0]*pow(r,2)+bbs.radial_distortion_coeff[1]*pow(r,4)))*u_norm;
    v_undist = (1/(1+bbs.radial_distortion_coeff[0]*pow(r,2)+bbs.radial_distortion_coeff[1]*pow(r,4)))*v_norm;

    //project to camera frame
    lambda = bbs.cam_offset[2]+bbs.plate_height+bbs.ball_radius;

    //project to world frame
    x_w =(-1)*lambda*u_undist+bbs.cam_offset[0];
    y_w = (-1)*lambda*v_undist+bbs.cam_offset[1];
    z_w = lambda-bbs.cam_offset[2];

    //output
    *x_out = x_w;
    *y_out = y_w;

    
  return 0;
};


//_______________________________________________________________________________________________________________________________
double discreteDerivative(const double dt, const double *x)
{
  // TODO: Implement a discrete derivative function
  //use array values 1 and 2 as maybe the first two values are identical due to pushback copying
  return (x[2]-x[1])/dt;
};


//_______________________________________________________________________________________________________________________________
double movingAverage(const int n, const double *raw_data)
{
  double sum =0;
  
  for(int i=0; i<n; i++){
      sum += raw_data[i];
  }
return (sum/n);
};


//_______________________________________________________________________________________________________________________________
double butterWorth(const double *x, const double *y)
{
  // TODO: Implement this if you like bonus points (not required to reach max points)
  /* ********************* */
  /* Insert your Code here */
  /* ********************* */
  return 0;
};



//_______________________________________________________________________________________________________________________________
int stepResponse(const double current_time, double *x_ref, double *y_ref,
                 double *vx_ref, double *vy_ref)
{

  double step_start = 5;
  double step_distance = 80;
  if (current_time < step_start)
  {
    *x_ref = 0;
  }
  else
  {
    *x_ref = step_distance;
  }

  *y_ref = 0;
  *vx_ref = 0;
  *vy_ref = 0;

  return 0;
};


//_______________________________________________________________________________________________________________________________
int circularTrajectory(const double current_time, double *x_ref, double *y_ref,
                       double *vx_ref, double *vy_ref)
{

  double traj_start = 3;
  double num_of_traj = 5; 
  double period = 4; 
  double R = 80;     // radius
  double t = current_time - traj_start;

  //wait 3 seconds in the center and do approx. 4 full circles
  if (current_time > traj_start && t < 4*4){
    *x_ref = R*cos(t*(M_PI)*2/period);
    *y_ref = R*sin(t*(M_PI)*2/period);
    *vx_ref = -R*sin(t*(M_PI)*2/period);
    *vy_ref = R*cos(t*(M_PI)*2/period);
  }
  else //stay in the center
  {
    *x_ref = 0;
    *y_ref = 0;
    *vx_ref = 0;
    *vy_ref = 0;
  }

  return 0;
};

// Don't change any of the below functions.



//_______________________________________________________________________________________________________________________________
int initBallBalancingRobot(int fd)
{
  // don't change this function.
  printf("\e[1;1H\e[2J"); // Clear screen
  printf("#######################\n");
  printf("Hardware Initialization\n");
  printf("#######################\n");

  // Let feather reboot
  usleep(200);

  // Check pixy readings
  int pixy_return, flag, x, y;
  pixy_return = readFromPixy(fd, &flag, &x, &y);
  while (!pixy_return)
  {
    // retry until connection established
    pixy_return = readFromPixy(fd, &flag, &x, &y);
  }

  printf("%s, x = %d, y = %d \n", "INIT: Pixy Coordinates received", x, y);

  // Do some inverse kinematics
  double position[] = {0, 0, 130};
  double plate_angles[] = {0, 0};
  double servo_angles[] = {0, 0, 0};
  inverseKinematics(plate_angles, servo_angles);
  printf("INIT: Inverse Kinematics initialized\n");
  printf("INIT: Finished\n");
  // printf("INIT: A: %.2f, B: %.2f, C: %.2f
  // \n",servo_angles[0],servo_angles[1],servo_angles[2]);

  tcflush(fd, TCIFLUSH);

  // Send motor commands
  // servoCommand(fd,servo_angles);

  return 1;
}



//_______________________________________________________________________________________________________________________________
/* Sends servo angles to serial port */
int servoCommand(int fd, double *servo_angles)
{
  // check serial
  int writeval;

  // assign values
  double angleA = servo_angles[0] + servo.bias_A;
  double angleB = servo_angles[1] + servo.bias_B;
  double angleC = servo_angles[2] + servo.bias_C;

  int min = servo.min_angle;
  int max = servo.max_angle;

  // check if values are valid
  int condition = (angleA < max && angleA > min) &&
                  (angleB < max && angleB > min) &&
                  (angleC < max && angleC > min);

  if (condition != 1)
  {
    printf("ERROR: Servo angles out of bounds.\n");
    return -1;
  }

  // assemble command
  char command[50];
  sprintf(command, "C %.2f %.2f %.2f\n", angleA, angleB, angleC);

  // Flush serial port output
  tcflush(fd, TCOFLUSH);
  // send command
  writeval = write(fd, command, strlen(command));
  usleep(1200); // wait for write function to complete

  return 0;
}




//_______________________________________________________________________________________________________________________________
/* Reads pixel coordinates from Pixycam. Also returns a flag whether an object
 * was detected or not */
int readFromPixy(int fd, int *flag, int *x, int *y)
{
  char buff[20];
  const char command[] = "P\n";
  int writeval;
  char *token;
  const char delim[] = " ";

  // Flush serial port input
  tcflush(fd, TCIFLUSH);
  tcflush(fd, TCOFLUSH);

  // Write command to pixy
  writeval = serialport_write(fd, command);
  usleep(10 * 1000);

  // Read until until no more bytes available
  // If not data is availabe, retry until success
  int readval = 0;
  while (readval != 1)
  {
    readval = serialport_read_until(fd, buff, sizeof(buff), '\n', 100);
  }

  // printf("readFromPixy: after read \n");
  // printf("writeval = %d, readval = %d", writeval,readval);

  // Catch read write errors
  if (!readval)
  {
    // printf("SERIAl READ FAILED with %d \n",readval);
    return -1;
  }

  // Add terminating 0 to make string
  buff[sizeof(buff) - 1] = 0;

  // extract values using strtok
  token = strtok(buff, delim);

  // Verify initial character
  if (token[0] != 'A')
  {
    // printf("SERIAL HEADER ERROR: %.20s\n",buff);
    return -1;
  }

  token = strtok(NULL, delim);
  *flag = atoi(token);
  token = strtok(NULL, delim);
  token = strtok(NULL, delim);
  *x = atoi(token);
  token = strtok(NULL, delim);
  token = strtok(NULL, delim);
  *y = atoi(token);

  return 1;
}



//_______________________________________________________________________________________________________________________________
long getMicroseconds()
{
  struct timeval the_time;
  long microseconds;
  gettimeofday(&the_time, NULL);
  microseconds = the_time.tv_sec * 1000000 + the_time.tv_usec / 1000 + the_time.tv_usec;      //wert "microseconds" ist anzahl mikrosekunden die seit 1 januar 1970 vergangen sind
  return microseconds;
}


//_______________________________________________________________________________________________________________________________
int pushBack(const double x_new, double *x_array, int array_size)
{
  for (int i = array_size - 1; i > 0; i = i - 1)
  {
    x_array[i] = x_array[i - 1];
  }

  x_array[0] = x_new;
  return 1;
};


//_______________________________________________________________________________________________________________________________
int startLogging(FILE *fp, int task_selection, double k_p, double k_d,
                 double k_i, int n_pos, int n_vel)
{
  fprintf(fp, "#Task Selector: %d\n", task_selection);
  fprintf(fp, "#PID Parameters: kp: %.2f, kd: %.2f, ki: %.2f\n", k_p, k_d, k_i);
  fprintf(fp, "#n_pos: %d\n", n_pos);
  fprintf(fp, "#n_vel: %d\n", n_vel);
  fprintf(fp,
          "t x_ref y_ref vx_ref vy_ref x_raw y_raw x y vx_raw vy_raw vx vy\n");
  fclose(fp);

  return 0;
}


//_______________________________________________________________________________________________________________________________
int logger(FILE *fp,
           long end_time,
           double current_time,
           double dt,
           double k_p,
           double k_d,
           double k_i,
           double x_ref,
           double y_ref,
           double vx_ref,
           double vy_ref,
           double x_raw,
           double y_raw,
           double x,
           double y,
           double vx_raw,
           double vy_raw,
           double vx,
           double vy,
           double *plate_angles,
           double *servo_angles,
           double x_integrator,
           double y_integrator)
{

  printf("\e[1;1H\e[2J");
  printf("##################################\n");
  printf("############# PID LOOP ###########\n");
  printf("##################################\n");
  printf("Timing\n");
  printf("Elapsed Time: %.2f s\n", current_time);
  printf("Step Time: %.4f s\n", dt);
  printf("Frequency: %.1f Hz\n\n", 1 / dt);
  printf("PID Parameters\n");
  printf("k_p: %.2f, k_d: %.2f, k_i: %.2f\n\n", k_p, k_d, k_i);
  printf("ReferencePosition\n");
  printf("x_ref: %.f \t y_ref: %.f\n\n", x_ref, y_ref);
  printf("Position\n");
  printf("x: %.f \t y: %.f\n\n", x, y);
  printf("Reference Velocity\n");
  printf("vx_ref: %.f \t vy_ref: %.f\n\n", vx_ref, vy_ref);
  printf("Velocity\n");
  printf("vx: %.f \t vy: %.f\n\n", vx, vy);
  printf("Plate Angles\n");
  printf("pitch: %.f deg, roll: %.f deg \n\n", plate_angles[1], plate_angles[0]);
  printf("Servo Angles\n");
  printf("A: %.f deg, B: %.f deg, C: %.f deg\n\n", servo_angles[0], servo_angles[1], servo_angles[2]);
  printf("Integrators\n");
  printf("x_integ: %.2f, y_integ: %.2f\n", x_integrator, y_integrator);

  fprintf(
      fp,
      "%lu %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
      end_time, x_ref, y_ref, vx_ref, vy_ref, x_raw, y_raw, x, y, vx_raw,
      vy_raw, vx, vy);
  fclose(fp);

  return 0;
};
