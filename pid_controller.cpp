/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <limits.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  output_lim_max = output_lim_maxi;
  output_lim_min = output_lim_mini;
  pro_error = 0.0;
  dif_error = 0.0;
  int_error = 0.0;
  prev_error = 0.0;
  total_error = 0.0;

  p_array[0] = Kpi;
  p_array[1] = Kdi;
  p_array[2] = Kii;
  dp_array[0] = 1;
  dp_array[1] = 1;
  dp_array[2] = 1;
  best_error = 99999999;

}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/

  pro_error = cte;
  dif_error = (cte - prev_error) / delta_time;
  int_error += cte * delta_time;
  prev_error = cte;

}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */

  while(dp_array[0] + dp_array[1] + dp_array[2] > output_lim_min){

    for(int i = 0; i < 3; i++)
    {
      p_array[i] += dp_array[i];
      total_error = (p_array[0] * pro_error) + (p_array[1] * dif_error) +  (p_array[2] * int_error);

      
      if(total_error < best_error)
      {
        best_error = total_error;
        dp_array[i] *= 1.1;
      }
      else
      {
        p_array[i] -= 2*dp_array[i];
        total_error = (p_array[0] * pro_error) + (p_array[1] * dif_error) +  (p_array[2] * int_error);
        
        if(total_error < best_error)
        {
          best_error = total_error;
          dp_array[i] *= 1.1;
        }
        else
        {
          p_array[i] += dp_array[i];
          dp_array[i] *= 0.9;
        }
      }
    }

  }
  
  double control = min(total_error, best_error);  
  
  return max(min(control, output_lim_max), output_lim_min);
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  delta_time = new_delta_time;

  return delta_time;
}