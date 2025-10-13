#pragma once

class PID {
public:
  PID();
  ~PID();

  // Initialize controller gains and output limits
  void Init(double k_p,
            double k_i,
            double k_d,
            double lim_max_output,
            double lim_min_output);

  // Set the controller timestep Δt (seconds)
  double UpdateDeltaTime(double new_delta_time);

  // Update internal PID error terms with the new input (cte-like signal)
  void UpdateError(double cte);

  // Evaluate the PID control output (clamped to limits)
  double TotalError();

  // Optional: clear integral term if needed by caller
  void ResetIntegral() { error_i = 0.0; }

private:
  // Gains
  double k_p{0.0};
  double k_i{0.0};
  double k_d{0.0};

  // Output clamp
  double lim_max_output{1.0};
  double lim_min_output{-1.0};

  // State
  double delta_t{0.0};

  // PID terms
  double error_p{0.0};
  double error_i{0.0};
  double error_d{0.0};

  // For proper derivative (previous proportional error)
  double prev_error_p{0.0};

  // Anti-windup for integral term
  double lim_max_integral{0.5};

  // Derivative low-pass filter (seconds). 0 => no filtering.
  double d_filter_tau{0.15};
  double d_filtered{0.0};
  bool   d_initialized{false};
};