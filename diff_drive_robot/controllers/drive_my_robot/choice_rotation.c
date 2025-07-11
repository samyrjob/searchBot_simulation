

// it's a modular function for the main controller to choose between turning right or left






#include <stdio.h>
#include <string.h>
#include <math.h>
#include "choice_rotation.h"




const char *rotate(double distance_left, double distance_right) {



  if (distance_left >= distance_right){
     return "left";
  }
  else {
     return "right";
  }



}



