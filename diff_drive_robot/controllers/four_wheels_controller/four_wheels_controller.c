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
#define FOV_STEP 0.01
#define ANGLE_STEP 0.05


 // motor wheels
static WbDeviceTag wheels[4];

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */


 // Convert axis-angle to quaternion
void axis_angle_to_quat(double axis[3], double angle, double q[4]) {
    double half = angle / 2.0;
    double sin_half = sin(half);
    q[0] = cos(half);
    q[1] = axis[0] * sin_half;
    q[2] = axis[1] * sin_half;
    q[3] = axis[2] * sin_half;
}

// Multiply two quaternions: q = q1 * q2
void quat_multiply(double q1[4], double q2[4], double q[4]) {
    q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    q[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    q[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    q[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

// Convert quaternion to axis-angle for Webots
void quat_to_axis_angle(double q[4], double axis_angle[4]) {
    double sin_half_angle = sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (sin_half_angle < 1e-6) {
        axis_angle[0] = 1.0;
        axis_angle[1] = 0.0;
        axis_angle[2] = 0.0;
        axis_angle[3] = 0.0;
    }
    else {
        axis_angle[0] = q[1] / sin_half_angle;
        axis_angle[1] = q[2] / sin_half_angle;
        axis_angle[2] = q[3] / sin_half_angle;
        axis_angle[3] = 2.0 * acos(q[0]);
    }
}






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






    // Get camera node
    WbNodeRef cam_node = wb_supervisor_node_get_from_def("POSE_CAMERA");
    WbFieldRef rotation_field = wb_supervisor_node_get_field(cam_node, "rotation");


    // get the camera device
    WbDeviceTag camera = wb_robot_get_device("camera_front");
    double fov = wb_camera_get_fov(camera);
    double min_fov = wb_camera_get_min_fov(camera);
    double max_fov = wb_camera_get_max_fov(camera);


    // Rotation angles
    double pitch = 0.0;  // up/down (around Y)
    double roll = 0.0;   // tilt left/right (around X)



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

      /*  printf("Initial FOV: %f\n", fov);

        printf("Min FOV: %f\n", min_fov);

        printf("Max FOV: %f\n", max_fov);*/

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

            // Zoom in/out
            if (key == 'Z' || key == 'z') {
                fov -= FOV_STEP;
                if (fov < 0.1) fov = 0.1;
                wb_camera_set_fov(camera, fov);
            }
            if (key == 'S' || key == 's') {
                fov += FOV_STEP;
                if (fov > 1.7) fov = 1.7;
                wb_camera_set_fov(camera, fov);
            }



            //    if (roll > 1.55) roll = 1.55;




         

            //    if (roll < -1.55) roll = -1.55;

        



            // Update pitch (A/E)
            if (key == 'A' || key == 'a') pitch += ANGLE_STEP;
            if (key == 'E' || key == 'e') pitch -= ANGLE_STEP;
            if (pitch > 1.5) pitch = 1.5;
            if (pitch < -0.4) pitch = -0.4;

            // Update roll (D/Q)
            if (key == 'D' || key == 'd') roll += ANGLE_STEP;
            if (key == 'Q' || key == 'q') roll -= ANGLE_STEP;
            // Compute combined pitch + roll rotation using quaternion math
            double pitch_axis[3] = { 1, 0, 0 };  // X axis
            double roll_axis[3] = { 0, 0, 1 };   // Z axis

            double pitch_q[4], roll_q[4], combined_q[4], final_rot[4];

            axis_angle_to_quat(pitch_axis, pitch, pitch_q);
            axis_angle_to_quat(roll_axis, roll, roll_q);

            // Combine them: apply roll *then* pitch (change order if needed)
            quat_multiply(roll_q, pitch_q, combined_q);
            quat_to_axis_angle(combined_q, final_rot);

            // Apply combined rotation to Webots object
            wb_supervisor_field_set_sf_rotation(rotation_field, final_rot);
        }

        // Apply final speed
        set_wheel_speeds(left_speed, right_speed);

        // Combine pitch and roll using a simplified logic:
       // 1. Apply pitch (Y axis)
       // 2. Then apply roll (X axis)


       

        printf("roll angle value equals to: %f \n ", roll);

        // Then: roll rotation
        // NOTE: This will override the above unless we compose both (see next)
        
        // ❗Only one rotation can be set like this — for full combo, we need quaternion math




         int count = wb_camera_recognition_get_number_of_objects(camera);
         const WbCameraRecognitionObject* objects = wb_camera_recognition_get_objects(camera);

         for (int i = 0; i < count; ++i) {
             int object_id = objects[i].id;
             WbNodeRef object_node = wb_supervisor_node_get_from_id(object_id);
             const char* model = objects[i].model;
             printf("Detected object type (model): %s\n", model);

             if (object_node) {
                 WbFieldRef name_field = wb_supervisor_node_get_field(object_node, "name");

                 if (name_field) {
                     const char* name = wb_supervisor_field_get_sf_string(name_field);
                     if (name)
                         printf("Detected object with name: %s\n", name);
                 }


                const double* position = wb_supervisor_node_get_position(object_node);
                printf("Object %d world coordinates: x=%.2f, y=%.2f, z=%.2f\n",
                    object_id, position[0], position[1], position[2]);
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