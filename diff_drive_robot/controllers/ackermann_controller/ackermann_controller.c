/*
 * File:          ackermann_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/keyboard.h>






/*
 * You may want to add macros here.
 */
#define TIME_STEP 64




 // motor wheels
static WbDeviceTag wheels[4];


// for Ackerman:
static WbDeviceTag steering[2];
static int has_steering = 0;



// set speed to wheels
void set_wheel_speeds(double left, double right) {
    wb_motor_set_position(wheels[0], INFINITY); // rear_right
    wb_motor_set_position(wheels[1], INFINITY); // rear_left
    wb_motor_set_position(wheels[2], INFINITY); // front_right
    wb_motor_set_position(wheels[3], INFINITY); // front_left
 

    wb_motor_set_velocity(wheels[0], right);
    wb_motor_set_velocity(wheels[1], left);
    wb_motor_set_velocity(wheels[2], right);
    wb_motor_set_velocity(wheels[3], left);

}


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  setvbuf(stdout, NULL, _IONBF, 0); // Disable stdout buffering
  wb_keyboard_enable(TIME_STEP);




  // all vehicles
  wheels[0] = wb_robot_get_device("rear_right_wheel");
  wheels[1] = wb_robot_get_device("rear_left_wheel");
  wheels[2] = wb_robot_get_device("front_right_wheel");
  wheels[3] = wb_robot_get_device("front_left_wheel");




  // only for Ackerman
  if (strcmp(wb_robot_get_name(), "ackerman") == 0) {
      has_steering = 1;
      steering[0] = wb_robot_get_device("front_left_steer");
      steering[1] = wb_robot_get_device("front_right_steer");
  }

  if (!steering[0]) printf("front_left_steer not found!\n");
  if (!steering[1]) printf("front_right_steer not found!\n");



 


  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */

  double time = 0.0;
  while (wb_robot_step(TIME_STEP) != -1) {
      time += TIME_STEP / 1000.0;


      //// Default: stop
      double left_speed = 0.0;
      double right_speed = 0.0;


      // default dir

      double dir = 0.0;

      // Collect all pressed keys
      int key;
      while ((key = wb_keyboard_get_key()) != -1) {
      


          if (key == WB_KEYBOARD_UP) {
              /*     left_speed += 6.0;
                   right_speed += 6.0;*/
              left_speed = 6.0;
              right_speed = 6.0;
          }
          if (key == WB_KEYBOARD_DOWN) {
              /*    left_speed -= 6.0;
                  right_speed -= 6.0;*/
              left_speed = -6.0;
              right_speed = -6.0;
          }

          // only for Ackerman
          if (has_steering) {
              /*double dir = 0.5 * sin(time);
              wb_motor_set_position(steering[0], dir);
              wb_motor_set_position(steering[1], dir);*/





              if (key == WB_KEYBOARD_RIGHT) {
                  dir = 0.5; // turn right
              }
              else if (key == WB_KEYBOARD_LEFT) {
                  dir = -0.5; // turn left
              }
              else {
                  dir = 0.0; // straight
              }



          }


      
      
      }


        wb_motor_set_position(steering[0], dir);
        wb_motor_set_position(steering[1], dir);
        set_wheel_speeds(left_speed, right_speed);
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
