/*
 * File:          four_wheels_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>s
#include <webots/camera.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64


 // motor wheels
static WbDeviceTag wheels[4];

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */



static void wheel_set_speed(WbDeviceTag tag, double speed) {
    if (speed >= 0.0) {
        wb_motor_set_position(tag, INFINITY);
        wb_motor_set_velocity(tag, speed);
    }
    else {
        wb_motor_set_position(tag, -INFINITY);
        wb_motor_set_velocity(tag, -speed);
    }
}


//static void wheels_set_speed(double speed) {
//    int i;
//    for (i = 0; i < 4; i++)
//        wheel_set_speed(wheels[i], speed);
//}

static void wheels_set_speed_front(double speed) {
    int i;
    for (i = 2; i < 4; i++)
        wheel_set_speed(wheels[i], speed);
}


static void wheels_set_speed_rear(double speed) {
    int i;
    for (i = 0; i < 2; i++)
        wheel_set_speed(wheels[i], speed);
}


int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

   // all vehicles
  wheels[0] = wb_robot_get_device("rear_right_wheel");
  wheels[1] = wb_robot_get_device("rear_left_wheel");
  wheels[2] = wb_robot_get_device("front_right_wheel");
  wheels[3] = wb_robot_get_device("front_left_wheel");







  // get the camera
  WbDeviceTag camera = wb_robot_get_device("camera_front");


  // enable the camera
  if (camera == 0) {
      printf("Camera not found!\n");
      return -1;
  }
  else {
      printf("Camera found!\n");
      wb_camera_enable(camera, TIME_STEP);
  }

  // go forward
  wheels_set_speed_front(3.0);
  wheels_set_speed_rear(1.5);


  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {

      //wheels_set_speed(2.0);


      const unsigned char* image = wb_camera_get_image(camera);

      wb_console_print("message\n");
      fflush(stdout);
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
