#pragma once

class PID {
public:
  PID();
  ~PID();

  // Initialize controller gains and output limits
  void init_controller(double k_p,
                       double k_i,
                       double k_d,
                       double lim_max_output,
                       double lim_min_output);

  // Update internal error terms with the new "cte"-like signal
  void update_error(double cte);

  // Evaluate the PID control output (clamped to limits)
  double total_error();

  // Update the controller timestep Δt (seconds)
  double update_delta_time(double new_delta_time);

  // Optional helper to clear integral windup
  void reset_integral() { error_i = 0.0; }

private:
  // Gains
  double k_p{0.0};
  double k_i{0.0};
  double k_d{0.0};

  // Output limits
  double lim_max_output{1.0};
  double lim_min_output{-1.0};

  // Current timestep
  double delta_t{0.0};

  // Error terms
  double error_p{0.0};
  double error_i{0.0};
  double error_d{0.0};

  // Keep the previous proportional error for derivative
  double prev_error_p{0.0};

  // Anti-windup clamp for the integral term
  double lim_max_integral{0.5};

  // Simple low-pass on derivative (seconds). 0 => no filter.
  double d_filter_tau{0.15};
  double d_filtered{0.0};
  bool   d_initialized{false};
};
