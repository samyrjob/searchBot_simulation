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
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/keyboard.h>

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






void set_wheel_speeds(double left, double right) {
    wb_motor_set_position(wheels[0], INFINITY); // rear_right
    wb_motor_set_position(wheels[1], INFINITY); // rear_left
    wb_motor_set_position(wheels[2], INFINITY); // front_right
    wb_motor_set_position(wheels[3], INFINITY); // front_left

    wb_motor_set_velocity(wheels[0], 3.0);
    wb_motor_set_velocity(wheels[1], 3.0);
    wb_motor_set_velocity(wheels[2], right);
    wb_motor_set_velocity(wheels[3], left);
}


int main(int argc, char** argv) {
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

    // get the IR sensor
    //WbDeviceTag IRsensor = wb_robot_get_device("ir_sensor");


    // enable the camera
    if (camera == 0) {
        printf("Camera not found!\n");
        return -1;
    }
    else {
        printf("Camera found!\n");
        wb_camera_enable(camera, TIME_STEP);
    }

    //enable the keyboard 
    wb_keyboard_enable(TIME_STEP);





    /* main loop
     * Perform simulation steps of TIME_STEP milliseconds
     * and leave the loop when the simulation is over
     */
    while (wb_robot_step(TIME_STEP) != -1) {

        //wheels_set_speed(2.0);


        const unsigned char* image = wb_camera_get_image(camera);
        int width = wb_camera_get_width(camera);
        int height = wb_camera_get_height(camera);

        int key = wb_keyboard_get_key();

        switch (key) {
        case WB_KEYBOARD_UP:
            set_wheel_speeds(3.0, 3.0); // forward
            break;
        case WB_KEYBOARD_DOWN:
            set_wheel_speeds(-3.0, -3.0); // backward
            break;
        case WB_KEYBOARD_LEFT:
            set_wheel_speeds(-2.0, 2.0); // turn left
            break;
        case WB_KEYBOARD_RIGHT:
            set_wheel_speeds(2.0, -2.0); // turn right
            break;
        case -1:
            set_wheel_speeds(0.0, 0.0); // stop
            break;



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

    }

        /* Enter your cleanup code here */

        /* This is necessary to cleanup webots resources */
        wb_robot_cleanup();

        return 0;
}