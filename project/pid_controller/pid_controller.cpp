/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "pid_controller.h"
#include <algorithm>

PID::PID() {}
PID::~PID() {}

void PID::Init(double k_p, double k_i, double k_d,
               double lim_max_output, double lim_min_output) {
  this->k_p = k_p;
  this->k_i = k_i;
  this->k_d = k_d;
  this->lim_max_output = lim_max_output;
  this->lim_min_output = lim_min_output;

  delta_t = 0.0;
  error_p = 0.0;
  error_i = 0.0;
  error_d = 0.0;
  prev_error_p = 0.0;

  // modest anti-windup and a tiny D filter
  lim_max_integral = 0.5;
  d_filter_tau = 0.15;
  d_filtered = 0.0;
  d_initialized = false;
}

void PID::UpdateError(double cte) {
  // keep previous P for derivative
  double prev_p = error_p;

  // proportional
  error_p = cte;

  // derivative (with a small low-pass for noise)
  if (delta_t > 0.0) {
    double raw_d = (error_p - prev_p) / delta_t;
    if (d_filter_tau > 0.0) {
      if (!d_initialized) { d_filtered = raw_d; d_initialized = true; }
      double alpha = delta_t / (d_filter_tau + delta_t);
      d_filtered = (1.0 - alpha) * d_filtered + alpha * raw_d;
      error_d = d_filtered;
    } else {
      error_d = raw_d;
    }
  } else {
    error_d = 0.0;
  }

  // integral with clamp
  error_i += cte * delta_t;
  if (error_i >  lim_max_integral) error_i =  lim_max_integral;
  if (error_i < -lim_max_integral) error_i = -lim_max_integral;
}

double PID::TotalError() {
  double u = k_p*error_p + k_d*error_d + k_i*error_i;
  if (u > lim_max_output) u = lim_max_output;
  if (u < lim_min_output) u = lim_min_output;
  return u;
}

double PID::UpdateDeltaTime(double new_delta_time) {
  delta_t = new_delta_time;
  return delta_t;
}


#endif //PID_CONTROLLER_H