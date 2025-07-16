/*
 * File:          drive_my_robot.c
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
#include <webots/motor.h>
#include <webots/camera.h>
#include <stdio.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <math.h>
#include "choice_rotation.h"
#include <webots/supervisor.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define MAX_SPEED 6.28

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

 
   // You should declare here WbDeviceTag variables for storing
 
   WbDeviceTag right_motor = wb_robot_get_device("motor1");
   WbDeviceTag left_motor = wb_robot_get_device("motor2");
   WbDeviceTag camera = wb_robot_get_device("camera");
   WbDeviceTag sensor_front = wb_robot_get_device("sensor_front");
   WbDeviceTag sensor_right = wb_robot_get_device("sensor_right");
   WbDeviceTag sensor_left = wb_robot_get_device("sensor_left");
   
   
     // Get the robot node by its DEF name
   WbNodeRef robot_node = wb_supervisor_node_get_from_def("Main_Robot");
   
   
  if (robot_node == NULL) {
    printf("Error: Could not find node with DEF MAIN_ROBOT.\n");
    wb_robot_cleanup();
    return 1;
  }
  
    // Get the 'rotation' field of the robot
  WbFieldRef rotation_field = wb_supervisor_node_get_field(robot_node, "rotation");

  if (rotation_field == NULL) {
    printf("Error: Could not find 'rotation' field in MAIN_ROBOT.\n");
    wb_robot_cleanup();
    return 1;
  }
  
  
  
   
     
     
   
   
   
   
   
    // enable the camera
    wb_camera_enable(camera, TIME_STEP);
    
    //enable the sensor
    wb_distance_sensor_enable(sensor_front, TIME_STEP);
    wb_distance_sensor_enable(sensor_right, TIME_STEP);
    wb_distance_sensor_enable(sensor_left, TIME_STEP);
   
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(right_motor, 0.0);
    
      
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
  

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
   
     
    // SPEED HERE :  
    double right_speed = MAX_SPEED*0.5;
    double left_speed = MAX_SPEED*0.5;
    
    
      
  
    
    
  while (wb_robot_step(TIME_STEP) != -1) {
  
    const unsigned char *image = wb_camera_get_image(camera); 
    
    // read the value of distance sensor_front
    double distance_front = wb_distance_sensor_get_value(sensor_front);
    double distance_right = wb_distance_sensor_get_value(sensor_right);
    double distance_left = wb_distance_sensor_get_value(sensor_left);
    
     // 📌 READ the current rotation
    const double *current_rotation = wb_supervisor_field_get_sf_rotation(rotation_field);
  
    //printf("Current rotation: axis == (%f, %f, %f), angle = %f radians\n",
           //current_rotation[0], current_rotation[1], current_rotation[2], current_rotation[3]);
           
           
  
  
    


     
     
       // Print the distance to the console
    // printf("Distance front: %f\n", distance_front);
    // printf("Distance right: %f\n", distance_right);
    // printf("Distance left: %f\n", distance_left);
    
    
    if (distance_front >= 1000) {
      wb_motor_set_velocity(right_motor, right_speed);
      wb_motor_set_velocity(left_motor, left_speed);
    }
    else {
    
      const char *direction = rotate(distance_left, distance_right);
      
      
          
      if(strcmp("left", direction)==0){
      
        // 📌 Optionally set a new rotation: 90 degrees (π/2) around Z-axis
        double new_rotation[4] = {
          // current_rotation[0], // x
          // current_rotation[1], // y
          // current_rotation[2], // z
          0,
          0,
          1,
          current_rotation[3] + M_PI /2
        
        };
        wb_supervisor_field_set_sf_rotation(rotation_field, new_rotation);
        
       wb_motor_set_velocity(right_motor, 0);
       wb_motor_set_velocity(left_motor, 0);
      
       // wb_motor_set_velocity(right_motor, -right_speed);
       // wb_motor_set_velocity(left_motor, left_speed);
      
      } 
      else if (strcmp("right", direction)==0){

        
           // 📌 Optionally set a new rotation: 45 degrees (π/4) around Z-axis
        double new_rotation_right[4] = {
          // current_rotation[0], // x
          // current_rotation[1], // y
          // current_rotation[2], // z
          0,
          0,
          1,
           current_rotation[3] + M_PI / 4
        
        };
      
        wb_supervisor_field_set_sf_rotation(rotation_field, new_rotation_right);
      
      
       // wb_motor_set_velocity(right_motor, right_speed);
       // wb_motor_set_velocity(left_motor, -left_speed);
    
    }
    
    
    }
     
     
     
 
    
    
    
    
    
    
    
    
    
    
     // Basic decision logic
     
     /*
    if (distance_front < 100.0) {
      printf("Obstacle ahead! Stop or turn.\n");
            // Set opposite speeds to rotate in place
      wb_motor_set_velocity(right_motor, 2.0);  // Forward
      wb_motor_set_velocity(left_motor, -2.0); // Backward
      // Stop after turning
       // SPEED HERE :  
      right_speed = 0;
      left_speed = 0;
             
      }
      else {
      printf("Path is clear. Move forward.\n");
    }*/
    
    
     
 

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
