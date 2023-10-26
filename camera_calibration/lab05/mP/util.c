#include "util.h"
#include "newton_raphson.h"
#include "math.h"
#define pi 3.141


int initBallBalancingRobot(int fd) {
  
  printf("\e[1;1H\e[2J"); // Clear screen
  printf("#######################\n");
  printf("Hardware Initialization\n");
  printf("#######################\n");

  // Let feather reboot
  usleep(200);

  // Check pixy readings
  int pixy_return, flag, x, y;
  pixy_return = readFromPixy(fd, &flag, &x, &y);
  while (!pixy_return) {
    // retry until connection established
    pixy_return = readFromPixy(fd, &flag, &x, &y);
  }
  
  printf("%s, x = %d, y = %d \n", "INIT: Pixy Coordinates received", x, y);

  // Do some inverse kinematics to check for errors
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

int inverseKinematics(const double* plate_angles_1, double* servo_angles) {
    // Load parameters R, L_1, L_2, P_z etc. from parameters file. Example: double R = bbs.R_plate_joint;
    // Then implement inverse kinematics similar to prelab
    
    /* ********************* */
  
  float R = bbs.R_plate_joint;
  float L1 = bbs.l1;  // short arms
  float L2 = bbs.l2;
  float P_z = bbs.plate_height;
  float beta[3];            //Initialize array to store beta servo angles
  float delta_z[3];
  double plate_angles[2];

 
  /*--------------------------------------------------------------------------
  
   Transform plate angles from degrees to radians for calculation

  --------------------------------------------------------------------------*/
  for(int i = 0; i<2; i++){
    plate_angles[i] = ( plate_angles_1[i] * pi ) / 180 ;
    
  }
 
  
  /*--------------------------------------------------------------------------
  
  // Calculate the z-offsets of each anchor point according to the formulas
  
  --------------------------------------------------------------------------*/

  delta_z[0]  = R * sin(plate_angles[0]);
  delta_z[1]  = -0.5 * R * sin(plate_angles[0]) + sqrt(3.0) / 2.0 * R * sin(plate_angles[1]);
  delta_z[2]  = -0.5 * R * sin(plate_angles[0]) - sqrt(3.0) / 2.0 * R * sin(plate_angles[1]);


  
    for(int i = 0; i<3; i++){
     // Calculate the beta-servo angles according to the law of cosines
     beta[i] = (pow(P_z +  delta_z[i], 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * (P_z +  delta_z[i])); 
     // Return -1 if any of the angles is not defined
    if(beta[i] < -1|| beta[i] >1) return -1;

     // Calculate the alpha servo angles and transform them back to degrees
     servo_angles[i] = (pi/2 - acos(beta[i]))*180/pi;
      
       }
  
  
  return 0;
    /* ********************* */

    
 //return -1; // if invalid input angle
  return 0; // if ok
};


int project2worldFrame(const int x_in, const int y_in, double* x_out, double* y_out){

    // implement the code to project the coordinates in the image frame to the world frame
    // make sure to multiply the raw pixy2 coordinates with the scaling factor (ratio between
    // image fed to python for calibration and pixy2 resolution): bbs.calibration_image_scale.
    
    /* ********************* */

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




/* Sends servo angles to serial port */
int servoCommand(int fd, double* servo_angles) {
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

  if (condition != 1) {
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

  return 0;
}

/* Reads pixel coordinates from Pixycam. Also returns a flag whether an object
 * was detected or not */
int readFromPixy(int fd, int* flag, int* x, int* y) {
  char buff[20];
  const char command[] = "P\n";
  int writeval;
  char* token;
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
  while(readval != 1){
    readval = serialport_read_until(fd, buff, sizeof(buff), '\n', 100);
  }
  
  // printf("readFromPixy: after read \n");
  // printf("writeval = %d, readval = %d", writeval,readval);

  // Catch read write errors
  if (!readval) {
    //printf("SERIAl READ FAILED with %d \n",readval);
    return -1;
  }

  // Add terminating 0 to make string
  buff[sizeof(buff) - 1] = 0;

  // extract values using strtok
  token = strtok(buff, delim);

  // Verify initial character
  if (token[0] != 'A') {
    //printf("SERIAL HEADER ERROR: %.20s\n",buff);
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

