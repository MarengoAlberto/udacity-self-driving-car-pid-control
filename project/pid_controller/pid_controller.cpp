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
    prop_cte = 0.0; // proportional errro
    diff_cte = 0.0; // differential error
    int_cte = 0.0; // integral error

    Kp = Kpi; // proportional coefficient
    Kd = Kdi; // differential coefficient
    Ki = Kii; // integral coefficient

    output_lim_min = output_lim_mini; // Minimal Output limit
    output_lim_max = output_lim_maxi; // Maximum Output limit

    dt = 0.0; // Delta time
}


void PID::UpdateError(double cte) {
    /**
    * TODO: Update PID errors based on cte.
    **/
    diff_cte = (dt > 0.0) ? (cte - prop_cte) / dt : 0.0;
    int_cte += cte * dt;
    prop_cte = cte;
}

double PID::TotalError() {
    /**
    * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
    */
   double control = (Kp * prop_cte) + (Kd * diff_cte) + (Ki * int_cte);

   control = max(min(control, output_lim_max), output_lim_min);

   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
    /**
    * TODO: Update the delta time with new value
    */
    dt = new_delta_time;

    return dt;
}