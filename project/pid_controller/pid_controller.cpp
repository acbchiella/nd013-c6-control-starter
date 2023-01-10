/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  _kp = Kpi;
  _ki = Kii;
  _kd = Kdi;
  _out_lim_max = output_lim_maxi;
  _out_lim_min = output_lim_mini;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  _cte_i += cte * _dt;
  
  _cte_d = 0;
  if (_dt > __DBL_EPSILON__) {
   _cte_d = (cte - _cte) / _dt;
  };
 
  _cte = cte; 
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double control;
   control = - _kp * _cte - _kd * _cte_d - _ki * _cte_i;

   return std::max(std::min(control, _out_lim_max), _out_lim_min);
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  _dt = new_delta_time;
}