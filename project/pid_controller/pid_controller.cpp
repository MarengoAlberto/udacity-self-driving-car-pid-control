#include "pid_controller.h"
#include <algorithm>
#include <cmath>

PID::PID() {}
PID::~PID() {}

void PID::init_controller(double k_p,
                          double k_i,
                          double k_d,
                          double lim_max_output,
                          double lim_min_output) {
  this->k_p = k_p;
  this->k_i = k_i;
  this->k_d = k_d;

  this->lim_max_output = lim_max_output;
  this->lim_min_output = lim_min_output;

  // initialize state
  this->delta_t = 0.0;
  this->error_p = 0.0;
  this->error_i = 0.0;
  this->error_d = 0.0;
  this->prev_error_p = 0.0;

  // sensible defaults
  this->lim_max_integral = 0.5;  // keep I modest to avoid bias
  this->d_filter_tau     = 0.15; // derivative low-pass, seconds
  this->d_filtered       = 0.0;
  this->d_initialized    = false;
}

void PID::update_error(double cte) {
  // Save previous proportional error for derivative calculation
  double prev_p = this->error_p;

  // Proportional term
  this->error_p = cte;

  // Derivative term (with optional first-order low-pass)
  if (this->delta_t > 0.0) {
    double raw_d = (this->error_p - prev_p) / this->delta_t;
    if (this->d_filter_tau > 0.0) {
      if (!d_initialized) { d_filtered = raw_d; d_initialized = true; }
      double alpha = this->delta_t / (this->d_filter_tau + this->delta_t);
      d_filtered = (1.0 - alpha) * d_filtered + alpha * raw_d;
      this->error_d = d_filtered;
    } else {
      this->error_d = raw_d;
    }
  } else {
    this->error_d = 0.0;
  }

  // Integral term with anti-windup clamp
  this->error_i += cte * this->delta_t;
  if (this->error_i >  this->lim_max_integral) this->error_i =  this->lim_max_integral;
  if (this->error_i < -this->lim_max_integral) this->error_i = -this->lim_max_integral;

  // Persist last P for next round (not strictly needed now)
  this->prev_error_p = prev_p;
}

double PID::total_error() {
  double control = (this->k_p * this->error_p)
                 + (this->k_d * this->error_d)
                 + (this->k_i * this->error_i);

  if (control > this->lim_max_output) control = this->lim_max_output;
  if (control < this->lim_min_output) control = this->lim_min_output;
  return control;
}

double PID::update_delta_time(double new_delta_time) {
  this->delta_t = new_delta_time;
  return this->delta_t;
}
