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
#include <webots/supervisor.h>

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

    wb_motor_set_velocity(wheels[0], right);
    wb_motor_set_velocity(wheels[1], left);
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



    // enable the camera
    if (camera == 0) {
        printf("Camera not found!\n");
        return -1;
    }
    else {
        printf("Camera found!\n");
        wb_camera_enable(camera, TIME_STEP);
    }



    // Enable camera recognition
    wb_camera_recognition_enable(camera, TIME_STEP);

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

        // Default: stop
        double left_speed = 0.0;
        double right_speed = 0.0;

        // Collect all pressed keys
        int key;
        while ((key = wb_keyboard_get_key()) != -1) {
            if (key == WB_KEYBOARD_UP) {
                left_speed += 6.0;
                right_speed += 6.0;
            }
            if (key == WB_KEYBOARD_DOWN) {
                left_speed -= 3.0;
                right_speed -= 3.0;
            }
            if (key == WB_KEYBOARD_LEFT) {
                left_speed -= 1.5;  // Reduce left, increase right to turn
                right_speed += 1.5;
            }
            if (key == WB_KEYBOARD_RIGHT) {
                left_speed += 1.5;
                right_speed -= 1.5;
            }
        }

        // Apply final speed
        set_wheel_speeds(left_speed, right_speed);




         int count = wb_camera_recognition_get_number_of_objects(camera);
         const WbCameraRecognitionObject* objects = wb_camera_recognition_get_objects(camera);

         for (int i = 0; i < count; ++i) {
             int object_id = objects[i].id;
             WbNodeRef object_node = wb_supervisor_node_get_from_id(object_id);

             if (object_node) {
                 WbFieldRef name_field = wb_supervisor_node_get_field(object_node, "name");

                 if (name_field) {
                     const char* name = wb_supervisor_field_get_sf_string(name_field);
                     if (name)
                         printf("Detected object with name: %s\n", name);
                 }
             }
         }











            wb_console_print("message\n");
            fflush(stdout);
            







        };

        /* Enter your cleanup code here */

        /* This is necessary to cleanup webots resources */
        wb_robot_cleanup();

        return 0;
}