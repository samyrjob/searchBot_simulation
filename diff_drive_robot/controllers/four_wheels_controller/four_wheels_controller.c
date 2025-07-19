#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/keyboard.h>
#include <webots/supervisor.h>
#include <webots/speaker.h>
#include <stdbool.h>
#include <webots/radar.h>

/*
 macros here.s
 */
#define TIME_STEP 64
#define FOV_STEP 0.01
#define ANGLE_STEP 0.05
#define PLAY_INTERVAL 10000  // 10 seconds in milliseconds
//#define MAX_STEERING_ANGLE 0.5
//#define STEERING_STEP 0.05


// motor wheels
static WbDeviceTag wheels[4];


// for Ackerman:
static WbDeviceTag steering[2];
static int has_steering = 0;




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



void process_camera_recognition(WbDeviceTag camera, char* sound_path) {
    int count = wb_camera_recognition_get_number_of_objects(camera);
    const WbCameraRecognitionObject* objects = wb_camera_recognition_get_objects(camera);

    for (int i = 0; i < count; ++i) {
        int object_id = objects[i].id;
        WbNodeRef object_node = wb_supervisor_node_get_from_id(object_id);
        const char* model = objects[i].model;

        if (object_node) {
            WbFieldRef name_field = wb_supervisor_node_get_field(object_node, "name");

            if (name_field) {
                const char* name = wb_supervisor_field_get_sf_string(name_field);
                if (name && strcmp(name, "missing_person") == 0) {
                    sound_path = "reassure_message_searchbot.wav";
                }
            }

            const double* position = wb_supervisor_node_get_position(object_node);

           
            
            if ( camera == wb_robot_get_device("rotating_camera")) {
                printf("Object detected by the ROTATING CAMERA !\n");
                printf("Coordinates: x=%.2f, y=%.2f, z=%.2f\n",
                    position[0], position[1], position[2]);
            }
            else {
                printf("Object detected ! Coordinates: x=%.2f, y=%.2f, z=%.2f\n",
                    position[0], position[1], position[2]);
            }
        }
    }
}





   


int main(int argc, char** argv) {
    /* necessary to initialize webots stuff */
    wb_robot_init();


    // Get the speaker device by its name "speaker"
    WbDeviceTag speaker = wb_robot_get_device("speaker");
    printf("Speaker tag: %d\n", speaker);

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


    //double steering_angle = 0.0;  // range: -0.5 (left) to 0.5 (right)



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
        // Enable camera recognition
        wb_camera_recognition_enable(camera, TIME_STEP);
    }



  

    //enable the keyboard 
    wb_keyboard_enable(TIME_STEP);

    bool play_loop = false;
    int elapsed_time = 0;

    const char* sound_path = "searchbot_message.wav";


    // get rotational camera
    WbDeviceTag cam_motor = wb_robot_get_device("camera_motor");
    WbDeviceTag rotating_camera = wb_robot_get_device("rotating_camera");
   




    // enable the camera
    if (rotating_camera == 0) {
        printf("rotating Camera not found!\n");
        return -1;
    }
    else {
        printf("Rotating camera found!\n");
        wb_camera_enable(rotating_camera, TIME_STEP);
        // Enable camera recognition
        wb_camera_recognition_enable(rotating_camera, TIME_STEP);
    }


    // set position of the camera_motor
    wb_motor_set_position(cam_motor, INFINITY);

    // Rotate continuously
    wb_motor_set_velocity(cam_motor, 0.5);


    // get and enable the radar 
    WbDeviceTag radar = wb_robot_get_device("radar"); 
    if (radar == 0) {
        printf("Radar device not found!\n");
        return -1;
    }
    wb_radar_enable(radar, TIME_STEP);






    // main loop

    double time = 0.0;
    
    while (wb_robot_step(TIME_STEP) != -1) {
        time += TIME_STEP / 1000.0;


        if (wb_camera_get_image(camera) == NULL) {
            printf("Main camera failed during runtime. Switching to backup.\n");
            // Stop rotation of backup camera to act like a fixed one
            wb_motor_set_position(cam_motor, -0.5);  // Stop rotating
            wb_motor_set_velocity(cam_motor, 0.0);
        }







        int target_count = wb_radar_get_number_of_targets(radar);
        const WbRadarTarget* targets = wb_radar_get_targets(radar);

      /*  printf("Detected %d targets\n", target_count);*/

    /*    for (int i = 0; i < target_count; i++) {
            printf("Target %d: distance = %f, azimuth = %f, speed = %f, power = %f\n",
                i, targets[i].distance, targets[i].azimuth, targets[i].speed, targets[i].received_power);
        }*/


        // more professional message :

        if (target_count > 0) {
            printf("[RADAR] %d potential object(s) detected. Sending coordinates to rescue team...\n", target_count);
            for (int i = 0; i < target_count; i++) {
                printf("[RADAR] Target %d → Estimated Distance: %.2f m | Azimuth: %.2f rad | Relative Speed: %.2f m/s | Signal Strength: %.2f\n",
                    i + 1, targets[i].distance, targets[i].azimuth, targets[i].speed, targets[i].received_power);
            }
        }



  





        //// Default: stop
        double left_speed = 0.0;
        double right_speed = 0.0;


        // dir for steering
        double dir = 0.0;

        // Collect all pressed keys
        int key;
        while ((key = wb_keyboard_get_key()) != -1) {




            // Toggle playback on key 'T'
            if (key == 'T' || key == 't') {
                play_loop = !play_loop;
                if (play_loop) {
                    printf(">> Playback started.\n");
                }
                else {
                    printf(">> Playback stopped.\n");
                    wb_speaker_stop(speaker, NULL);  // stop all sounds
                    elapsed_time = 0;
                }
            }

         


      
            if (key == WB_KEYBOARD_UP) {
          
                left_speed = 6.0;
                right_speed = 6.0;
            }
            if (key == WB_KEYBOARD_DOWN) {
         
                left_speed = - 6.0;
                right_speed = - 6.0;
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




        



            // Update pitch (A/E)
            if (key == 'A' || key == 'a') pitch += ANGLE_STEP;
            if (key == 'E' || key == 'e') pitch -= ANGLE_STEP;
            if (pitch > 1.5) pitch = 1.5;
            if (pitch < -0.4) pitch = -0.4;

            // Update roll (D/Q)
            if (key == 'D' || key == 'd') roll += ANGLE_STEP;
            if (key == 'Q' || key == 'q') roll -= ANGLE_STEP;
            if (roll > 1.55) roll = 1.55;
            if (roll < -1.55) roll = -1.55;


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



        // apply steering

        wb_motor_set_position(steering[0], dir);
        wb_motor_set_position(steering[1], dir);
    
        // Apply final speed
        set_wheel_speeds(left_speed, right_speed);





        // apply sound settings

         // If playback is active, play sound every 10 seconds
        if (play_loop) {
            elapsed_time += TIME_STEP;
            if (elapsed_time >= PLAY_INTERVAL) {
                wb_speaker_play_sound(speaker, speaker, sound_path,
                    1.0,   // volume (0.0 to 1.0)
                    1.0,   // pitch (1.0 = default)
                    0.0,   // balance (-1.0 to 1.0, 0 = center)
                    false  // loop
                );
                elapsed_time = 0;
            }

        }




        process_camera_recognition(camera, sound_path);
        process_camera_recognition(rotating_camera, sound_path);



            







        };

        /* Enter your cleanup code here */

        /* This is necessary to cleanup webots resources */
        wb_robot_cleanup();

        return 0;
}