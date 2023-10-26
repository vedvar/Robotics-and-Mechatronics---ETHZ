#include "util.h"

// Allocate structs
parameter_struct bbs;
servo_struct servo;

/*Parameters related to robot hardware */
int load_parameters()
{
  // Dimensions measured in CAD
  // All lengths in [mm]
  bbs.plate_height = 100;
  bbs.R_plate_joint = 113.24;
  bbs.R_base_servo = 102.92;
  bbs.l1 = 42.0; // short arms
  bbs.l2 = 100.0;
  bbs.first_link_angle_bias = 4.6; // deg
  bbs.diameter_plate = 250.0;
  bbs.base_angles[0] = 0.5 * M_PI;          // rad
  bbs.base_angles[1] = (7.0 * M_PI) / 6.0;  // rad
  bbs.base_angles[2] = (11.0 * M_PI) / 6.0; // rad
  bbs.ball_radius = 20;

  // When pictures are taken in PixyMon, their resolution varies. This factor
  // accounts for this scaling, such that the bbs can be copied directly
  // from Matlab
  // For system "Gurten" (put your own parameters here!):
  //////////////////
  //// Gurten //////
  //////////////////
  bbs.calibration_image_scale = 3.51; 
  bbs.focal_length = 418; //495
  bbs.radial_distortion_coeff[0] = -0.2141; 
  bbs.radial_distortion_coeff[1] = 1.09; // used old value because it works better
  bbs.distortion_center[0] = 556; 
  bbs.distortion_center[1] = 327; 

  // Adjust these if lens is removed and reinserted
  bbs.cam_offset[0] = 4; 
  bbs.cam_offset[1] = 20; 
  bbs.cam_offset[2] = 28.1; 

  return 1;
}

/*Parameters related to the servo KST 05725MG. If another servo is used, these
 * parameters and the parameters of the microcontroller need to be adjusted.*/
int load_servo()
{
  // Set angle bounds to avoid damage
  servo.min_angle = -50;
  servo.max_angle = 70;

  // Calibration of the mounting offset. Should be redone e.g. if servo.arms were
  // modified

  //////////////////
  //// Gurten //////
  //////////////////
  servo.bias_A = -9; //-3
  servo.bias_B = 0;
  servo.bias_C = 0;

  //////////////////
  ////  Rigi  //////
  //////////////////
  // servo.bias_A = -3;
  // servo.bias_B = 1;
  // servo.bias_C = 3;

  //////////////////
  ////  Napf  //////
  //////////////////

  // servo.bias_A = -3;
  // servo.bias_B = 4;
  // servo.bias_C = -1;
  return 1;
}
