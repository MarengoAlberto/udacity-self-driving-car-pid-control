/* ------------------------------------------------------------------------------
 * Project "5.1: Control and Trajectory Tracking for Autonomous Vehicles"
 * This file runs the behavior/motion planner and the controllers.
 * ----------------------------------------------------------------------------
 */
#include "json.hpp"
#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"
#include "pid_controller.h"
#include "Eigen/QR"

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>

#include <uWS/uWS.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

using json = nlohmann::json;
#define _USE_MATH_DEFINES

/*** Globals provided by the starter ***/
BehaviorPlannerFSM behavior_planner(
    P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
    P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
    P_MAX_ACCEL, P_STOP_LINE_BUFFER);

MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
std::vector<State> obstacles;

/*** Helpers from the starter (unchanged) ***/
std::string has_data(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("{");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) return "";
  else if (b1 != std::string::npos && b2 != std::string::npos)
    return s.substr(b1, b2 - b1 + 1);
  return "";
}

void set_obst(std::vector<double> x_points,
              std::vector<double> y_points,
              std::vector<State>& obstacles,
              bool& obst_flag) {
  for (size_t i = 0; i < x_points.size(); ++i) {
    State obstacle;
    obstacle.location.x = x_points[i];
    obstacle.location.y = y_points[i];
    obstacles.push_back(obstacle);
  }
  obst_flag = true;
}

std::size_t get_closest_point_idx(double point_x, double point_y,
                                  std::vector<double> points_x,
                                  std::vector<double> points_y) {
  std::size_t closest_idx = 0;
  double dist_min = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < points_x.size(); ++i) {
    double dx = point_x - points_x[i];
    double dy = point_y - points_y[i];
    double dist = std::sqrt(dx*dx + dy*dy);
    if (dist < dist_min) { dist_min = dist; closest_idx = i; }
  }
  return closest_idx;
}

double angle_between_points(double x1, double y1, double x2, double y2) {
  return std::atan2(y2 - y1, x2 - x1);
}

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

/*** Path planner wrapper (starter’s logic, unchanged) ***/
void path_planner(std::vector<double>& x_points,
                  std::vector<double>& y_points,
                  std::vector<double>& v_points,
                  double yaw,
                  double velocity,
                  State goal,
                  bool is_junction,
                  std::string tl_state,
                  std::vector<std::vector<double>>& spirals_x,
                  std::vector<std::vector<double>>& spirals_y,
                  std::vector<std::vector<double>>& spirals_v,
                  std::vector<int>& best_spirals) {
  State ego_state;
  ego_state.location.x = x_points.back();
  ego_state.location.y = y_points.back();
  ego_state.velocity.x = velocity;
  if (x_points.size() > 1) {
    ego_state.rotation.yaw = angle_between_points(
        x_points[x_points.size()-2], y_points[y_points.size()-2],
        x_points[x_points.size()-1], y_points[y_points.size()-1]);
    ego_state.velocity.x = v_points.back();
    if (velocity < 0.01) ego_state.rotation.yaw = yaw;
  }

  Maneuver behavior = behavior_planner.get_active_maneuver();
  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

  if (behavior == STOPPED) {
    int max_points = 20;
    double px = x_points.back(), py = y_points.back();
    while ((int)x_points.size() < max_points) {
      x_points.push_back(px); y_points.push_back(py); v_points.push_back(0.0);
    }
    return;
  }

  auto goal_set = motion_planner.generate_offset_goals(goal);
  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);

  if (spirals.empty()) {
    std::cout << "Error: No spirals generated\n"; return;
  }

  auto desired_speed = utils::magnitude(goal.velocity);
  State lead_car_state;

  for (auto& sp : spirals) {
    auto traj = motion_planner._velocity_profile_generator
                  .generate_trajectory(sp, desired_speed, ego_state, lead_car_state, behavior);

    std::vector<double> sx, sy, sv;
    sx.reserve(traj.size()); sy.reserve(traj.size()); sv.reserve(traj.size());
    for (auto& t : traj) {
      sx.push_back(t.path_point.x);
      sy.push_back(t.path_point.y);
      sv.push_back(t.v);
    }
    spirals_x.push_back(std::move(sx));
    spirals_y.push_back(std::move(sy));
    spirals_v.push_back(std::move(sv));
  }

  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);

  int best_idx = -1;
  if (!best_spirals.empty()) best_idx = best_spirals.back();

  int index = 0, max_points = 20;
  int add_points = spirals_x[best_idx].size();
  while ((int)x_points.size() < max_points && index < add_points) {
    x_points.push_back(spirals_x[best_idx][index]);
    y_points.push_back(spirals_y[best_idx][index]);
    v_points.push_back(spirals_v[best_idx][index]);
    ++index;
  }
}

/*** MAIN ***/
int main() {
  std::cout << "Starting server\n";
  uWS::Hub h;

  // High-resolution clock for Δt
  using clock_t = std::chrono::steady_clock;
  auto prev_tp = clock_t::now();
  double new_delta_time = 0.0;

  int i = 0;

  // Reset logs (optional)
  { std::ofstream("steer_pid_data.txt", std::ofstream::trunc).close();
    std::ofstream("throttle_pid_data.txt", std::ofstream::trunc).close(); }

  // Controllers
  PID pid_throttle;
  pid_throttle.init_controller(0.1, 0.003, 0.4, 0.0, -0.5);

  PID pid_steer;
  // gentle PID around the steering target, clipped to ±0.60 rad
  pid_steer.init_controller(0.1, 0.003, 0.4, 0.0, -0.50);

  auto wrap = [](double a){
    while (a >  M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
  };

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    auto s = has_data(std::string(data, length));
    if (s.empty()) return;

    json in = json::parse(s);

    // (Re)open logs
    std::fstream file_steer("steer_pid_data.txt");
    std::fstream file_throttle("throttle_pid_data.txt");

    // Trajectory and waypoint
    std::vector<double> x_points = in["traj_x"];
    std::vector<double> y_points = in["traj_y"];
    std::vector<double> v_points = in["traj_v"];
    double waypoint_x = in["waypoint_x"];
    double waypoint_y = in["waypoint_y"];
    double waypoint_t = in["waypoint_t"];
    bool   is_junction = in["waypoint_j"];

    // Ego
    double x_position = in["location_x"];
    double y_position = in["location_y"];
    double yaw        = in["yaw"];
    double velocity   = in["velocity"];

    // Env
    std::string tl_state = in["tl_state"];

    // Refresh obstacles every tick (avoid stale obstacle bias)
    obstacles.clear();
    have_obst = false;
    {
      std::vector<double> x_obst = in["obst_x"];
      std::vector<double> y_obst = in["obst_y"];
      set_obst(x_obst, y_obst, obstacles, have_obst);
    }

    // Plan
    State goal;
    goal.location.x = waypoint_x;
    goal.location.y = waypoint_y;
    goal.rotation.yaw = waypoint_t;

    std::vector<std::vector<double>> spirals_x, spirals_y, spirals_v;
    std::vector<int> best_spirals;

    path_planner(x_points, y_points, v_points, yaw, velocity,
                 goal, is_junction, tl_state,
                 spirals_x, spirals_y, spirals_v, best_spirals);

    // Δt
    auto now = clock_t::now();
    new_delta_time = std::chrono::duration<double>(now - prev_tp).count();
    prev_tp = now;

    // ----------------------- STEERING -----------------------
    pid_steer.update_delta_time(new_delta_time);

    // Closest path point
    std::size_t idx = get_closest_point_idx(x_position, y_position, x_points, y_points);

    // Compute signed cross-track error using the forward segment
    std::size_t i0 = idx;
    std::size_t i1 = std::min(i0 + 1, x_points.size() - 1);
    double sx = x_points[i1] - x_points[i0];
    double sy = y_points[i1] - y_points[i0];
    double seg_len = std::hypot(sx, sy) + 1e-9;

    double vx = x_position - x_points[i0];
    double vy = y_position - y_points[i0];

    // Signed lateral error: positive if the car is LEFT of the segment direction
    double cte = (vx*sy - vy*sx) / seg_len;

    // Heading error to a lookahead point (helps with curvature)
    std::size_t look = std::min(i0 + 6, x_points.size() - 1);
    double desired_yaw = std::atan2(y_points[look] - y_position,
                                    x_points[look] - x_position);
    double epsi = wrap(desired_yaw - yaw);

    // Stanley-style target steering (heading + lateral)
    const double Kh = 0.8;   // heading weight
    const double Kcte = 1.2; // lateral weight
    double steer_target = Kh * epsi + std::atan2(Kcte * cte, velocity + 0.1);

    // Smooth target with small PID and clamp
    pid_steer.update_error(steer_target);
    double steer_output = pid_steer.total_error();
    if (steer_output >  0.60) steer_output =  0.60;
    if (steer_output < -0.60) steer_output = -0.60;

    // Log steering
    if (file_steer.good()) {
      file_steer.seekg(std::ios::beg);
      for (int j=0; j < i-1; ++j)
        file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      file_steer << i << " " << steer_target << " " << steer_output << "\n";
    }

    // --------------------- THROTTLE / BRAKE ---------------------
    pid_throttle.update_delta_time(new_delta_time);

    double error_throttle = v_points[idx] - velocity;
    pid_throttle.update_error(error_throttle);
    double t_resp = pid_throttle.total_error();

    double throttle_output = (t_resp > 0.0) ? t_resp : 0.0;
    double brake_output    = (t_resp > 0.0) ? 0.0   : -t_resp;

    if (file_throttle.good()) {
      file_throttle.seekg(std::ios::beg);
      for (int j=0; j < i-1; ++j)
        file_throttle.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      file_throttle << i << " " << error_throttle << " " << brake_output
                    << " " << throttle_output << "\n";
    }

    // ------------------------- SEND -------------------------
    json out;
    out["brake"] = brake_output;
    out["throttle"] = throttle_output;
    out["steer"] = steer_output;

    out["trajectory_x"] = x_points;
    out["trajectory_y"] = y_points;
    out["trajectory_v"] = v_points;
    out["spirals_x"] = spirals_x;
    out["spirals_y"] = spirals_y;
    out["spirals_v"] = spirals_v;
    out["spiral_idx"] = best_spirals;
    out["active_maneuver"] = behavior_planner.get_active_maneuver();

    out["update_point_thresh"] = 16;

    auto msg = out.dump();
    i += 1;

    file_steer.close();
    file_throttle.close();

    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  });

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!\n";
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected\n";
  });

  int port = 4567;
  if (h.listen("0.0.0.0", port)) {
    std::cout << "Listening to port " << port << "\n";
    h.run();
  } else {
    std::cerr << "Failed to listen to port\n";
    return -1;
  }
}
