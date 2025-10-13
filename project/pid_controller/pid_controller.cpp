#include "pid_controller.h"
#include <algorithm>

PID::PID() {}
PID::~PID() {}

void PID::Init(double k_p_, double k_i_, double k_d_,
               double lim_max_output_, double lim_min_output_) {
  k_p = k_p_; k_i = k_i_; k_d = k_d_;
  lim_max_output = lim_max_output_;
  lim_min_output = lim_min_output_;

  delta_t = 0.0;
  error_p = error_i = error_d = 0.0;
  prev_error_p = 0.0;

  lim_max_integral = 0.5;   // modest I clamp
  d_filter_tau     = 0.15;  // smooth D a bit
  d_filtered = 0.0;
  d_initialized = false;
}

double PID::UpdateDeltaTime(double new_dt) {
  delta_t = new_dt;
  return delta_t;
}

void PID::UpdateError(double cte) {
  // Keep previous P for derivative
  double prev_p = error_p;

  // Proportional
  error_p = cte;

  // Derivative (with optional low-pass)
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

  // Integral with clamp (anti-windup)
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
