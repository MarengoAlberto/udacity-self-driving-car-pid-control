/* ------------------------------------------------------------------------------
 * Project "5.1: Control and Trajectory Tracking for Autonomous Vehicles"
 * Authors     : Mathilde Badoual.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the PID controller.
 * ----------------------------------------------------------------------------
 */
// pid_controller.cpp
#include "pid_controller.h"
#include <algorithm>
#include <cmath>

PID::PID() {}
PID::~PID() {}

void PID::init_controller(double kp, double ki, double kd,
                          double lim_max, double lim_min,
                          double lim_I, double tau) {
  k_p = kp; k_i = ki; k_d = kd;
  lim_max_output = lim_max;
  lim_min_output = lim_min;
  lim_max_integral = std::abs(lim_I);
  d_filter_tau = std::max(0.0, tau);

  delta_t = 0.0;
  error_p = error_i = error_d = 0.0;
  prev_error_p = 0.0;
  d_filtered = 0.0;
  d_initialized = false;
}

void PID::update_error(double cte, bool allow_integrate) {
  // derivative uses previous proportional error
  prev_error_p = error_p;
  error_p = cte;

  if (delta_t > 0.0) {
    double raw_d = (error_p - prev_error_p) / delta_t;

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

  if (allow_integrate) {
    error_i += cte * delta_t;
    // clamp integral to avoid windup
    if (error_i >  lim_max_integral) error_i =  lim_max_integral;
    if (error_i < -lim_max_integral) error_i = -lim_max_integral;
  }
}

double PID::total_error() {
  double u = k_p*error_p + k_d*error_d + k_i*error_i;
  // hard clamp output
  if (u > lim_max_output) u = lim_max_output;
  if (u < lim_min_output) u = lim_min_output;
  return u;
}

double PID::update_delta_time(double new_dt) {
  delta_t = new_dt;
  return delta_t;
}