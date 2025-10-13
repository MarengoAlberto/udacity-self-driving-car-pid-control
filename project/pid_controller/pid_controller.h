#pragma once

class PID {
public:
  PID();
  ~PID();

  // Same API names you use in main.cpp
  void Init(double k_p, double k_i, double k_d,
            double lim_max_output, double lim_min_output);

  double UpdateDeltaTime(double new_delta_time);
  void   UpdateError(double cte);
  double TotalError();

  // Optional helper
  void   ResetIntegral() { error_i = 0.0; }

private:
  // Gains
  double k_p{0.0}, k_i{0.0}, k_d{0.0};

  // Output clamp
  double lim_max_output{1.0}, lim_min_output{-1.0};

  // Timing
  double delta_t{0.0};

  // PID states
  double error_p{0.0}, error_i{0.0}, error_d{0.0};
  double prev_error_p{0.0};

  // Anti-windup for I term
  double lim_max_integral{0.5};

  // Small derivative low-pass (seconds). 0 => no filter
  double d_filter_tau{0.15};
  double d_filtered{0.0};
  bool   d_initialized{false};
};
