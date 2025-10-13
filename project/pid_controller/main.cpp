/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 *              Aaron Brown
 **********************************************/

#include <string>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <fstream>
#include <typeinfo>
#include <limits>

#include "json.hpp"
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

#include "Eigen/QR"
#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"
#include "pid_controller.h"

#include <uWS/uWS.h>
#include <math.h>
#include <time.h>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES

// ---------- helpers ----------
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("{");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) return "";
  else if (b1 != string::npos && b2 != string::npos) return s.substr(b1, b2 - b1 + 1);
  return "";
}

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

double angle_between_points(double x1, double y1, double x2, double y2) {
  return atan2(y2 - y1, x2 - x1);
}

// ---------- global modules ----------
BehaviorPlannerFSM behavior_planner(P_LOOKAHEAD_TIME,
                                    P_LOOKAHEAD_MIN,
                                    P_LOOKAHEAD_MAX,
                                    P_SPEED_LIMIT,
                                    P_STOP_THRESHOLD_SPEED,
                                    P_REQ_STOPPED_TIME,
                                    P_REACTION_TIME,
                                    P_MAX_ACCEL,
                                    P_STOP_LINE_BUFFER);

MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;

// ---------- provided utils ----------
void set_obst(vector<double> x_points, vector<double> y_points,
              vector<State>& obstacles, bool& obst_flag) {
  for (size_t i = 0; i < x_points.size(); i++) {
    State obstacle;
    obstacle.location.x = x_points[i];
    obstacle.location.y = y_points[i];
    obstacles.push_back(obstacle);
  }
  obst_flag = true;
}

int find_closest_point_idx(double x_position, double y_position,
                           vector<double> x_points, vector<double> y_points) {
  int closest_point_idx = 0;
  double dis_min = std::numeric_limits<double>::max();
  for (int i = 0; i < (int)x_points.size(); ++i) {
    double dx = x_position - x_points[i];
    double dy = y_position - y_points[i];
    double act_dis = dx*dx + dy*dy;
    if (act_dis < dis_min) { dis_min = act_dis; closest_point_idx = i; }
  }
  return closest_point_idx;
}

double angle_wrap(double a) {
  while (a >  M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

// ---------- planner wrapper (unchanged logic) ----------
void path_planner(vector<double>& x_points,
                  vector<double>& y_points,
                  vector<double>& v_points,
                  double yaw,
                  double velocity,
                  State goal,
                  bool is_junction,
                  string tl_state,
                  vector<vector<double>>& spirals_x,
                  vector<vector<double>>& spirals_y,
                  vector<vector<double>>& spirals_v,
                  vector<int>& best_spirals) {
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
      x_points.push_back(px);
      y_points.push_back(py);
      v_points.push_back(0);
    }
    return;
  }

  auto goal_set = motion_planner.generate_offset_goals(goal);
  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);
  auto desired_speed = utils::magnitude(goal.velocity);
  State lead_car_state;

  if (spirals.empty()) { cout << "Error: No spirals generated\n"; return; }

  for (int i = 0; i < (int)spirals.size(); i++) {
    auto trajectory = motion_planner._velocity_profile_generator
        .generate_trajectory(spirals[i], desired_speed, ego_state, lead_car_state, behavior);

    vector<double> sx, sy, sv;
    sx.reserve(trajectory.size()); sy.reserve(trajectory.size()); sv.reserve(trajectory.size());
    for (auto& t : trajectory) {
      sx.push_back(t.path_point.x);
      sy.push_back(t.path_point.y);
      sv.push_back(t.v);
    }
    spirals_x.push_back(std::move(sx));
    spirals_y.push_back(std::move(sy));
    spirals_v.push_back(std::move(sv));
  }

  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);

  int best_spiral_idx = -1;
  if (!best_spirals.empty()) best_spiral_idx = best_spirals.back();

  int index = 0;
  int max_points = 20;
  int add_points = spirals_x[best_spiral_idx].size();
  while ((int)x_points.size() < max_points && index < add_points) {
    x_points.push_back(spirals_x[best_spiral_idx][index]);
    y_points.push_back(spirals_y[best_spiral_idx][index]);
    v_points.push_back(spirals_v[best_spiral_idx][index]);
    ++index;
  }
}

// ---------- MAIN ----------
int main() {
  cout << "starting server" << endl;
  uWS::Hub h;

  // High-resolution Δt
  using clock_t = std::chrono::steady_clock;
  auto prev_tp = clock_t::now();
  double new_delta_time = 0.0;

  int i = 0;

  // Reset logs
  { ofstream("steer_pid_data.txt", std::ofstream::trunc).close();
    ofstream("throttle_pid_data.txt", std::ofstream::trunc).close(); }

  // Initialize PIDs with your gains
  PID pid_steer;    pid_steer.Init(0.29, 0.0011, 0.71, 1.2, -1.2);
  PID pid_throttle; pid_throttle.Init(0.21, 0.001, 0.019, 1.0, -1.0);

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    auto s = hasData(string(data, length));
    if (s == "") return;

    auto msg = json::parse(s);

    // open logs
    fstream file_steer("steer_pid_data.txt");
    fstream file_throttle("throttle_pid_data.txt");

    // Trajectory and waypoint
    vector<double> x_points = msg["traj_x"];
    vector<double> y_points = msg["traj_y"];
    vector<double> v_points = msg["traj_v"];
    double waypoint_x = msg["waypoint_x"];
    double waypoint_y = msg["waypoint_y"];
    double waypoint_t = msg["waypoint_t"];
    bool   is_junction = msg["waypoint_j"];

    // Ego
    double x_position = msg["location_x"];
    double y_position = msg["location_y"];
    double yaw        = msg["yaw"];
    double velocity   = msg["velocity"];

    // Env
    string tl_state = msg["tl_state"];

    // -------- refresh obstacles each tick (ahead-only filter) --------
    obstacles.clear();
    have_obst = false;
    {
      vector<double> x_obst = msg["obst_x"];
      vector<double> y_obst = msg["obst_y"];

      double cos_y = std::cos(yaw), sin_y = std::sin(yaw);
      for (size_t k = 0; k < x_obst.size(); ++k) {
        double dx = x_obst[k] - x_position;
        double dy = y_obst[k] - y_position;
        double forward = dx * cos_y + dy * sin_y;
        double dist    = std::hypot(dx, dy);
        if (forward > -2.0 && dist < 30.0) { // ahead-ish & close
          State o; o.location.x = x_obst[k]; o.location.y = y_obst[k];
          obstacles.push_back(o); have_obst = true;
        }
      }
    }

    // -------- plan path (unchanged) --------
    State goal; goal.location.x = waypoint_x; goal.location.y = waypoint_y; goal.rotation.yaw = waypoint_t;
    vector<vector<double>> spirals_x, spirals_y, spirals_v;
    vector<int> best_spirals;
    path_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state,
                 spirals_x, spirals_y, spirals_v, best_spirals);

    // -------- Δt (high-res) --------
    auto now = clock_t::now();
    new_delta_time = std::chrono::duration<double>(now - prev_tp).count();
    prev_tp = now;

    // ===================== STEERING =====================
    pid_steer.UpdateDeltaTime(new_delta_time);

    // closest & lookahead
    int i0 = find_closest_point_idx(x_position, y_position, x_points, y_points);
    int look = std::min(i0 + 6, (int)x_points.size() - 1);

    // vector to lookahead in world frame
    double dx_w = x_points[look] - x_position;
    double dy_w = y_points[look] - y_position;

    // rotate to vehicle frame (x forward, y left)
    double cos_y = std::cos(yaw), sin_y = std::sin(yaw);
    double x_v =  cos_y * dx_w + sin_y * dy_w;   // forward
    double y_v = -sin_y * dx_w + cos_y * dy_w;   // lateral (+left, -right)

    // Pure Pursuit steering (wheelbase ~2.7 m)
    const double L = 2.7;
    double Ld2 = std::max(1e-3, x_v*x_v + y_v*y_v);
    double pp_cmd = std::atan2(2.0 * L * y_v, Ld2);

    // small heading blend (can set to 0.0 if you want pure PP)
    double desired_yaw = std::atan2(dy_w, dx_w);
    double epsi = angle_wrap(desired_yaw - yaw);
    double steer_target = pp_cmd + 0.15 * epsi;

    pid_steer.UpdateError(steer_target);
    double steer_output = pid_steer.TotalError();

    // optional actuator clamp to ±0.60 rad
    if (steer_output >  0.60) steer_output =  0.60;
    if (steer_output < -0.60) steer_output = -0.60;

    // log steer
    if (file_steer.good()) {
      file_steer.seekg(std::ios::beg);
      for (int j = 0; j < i - 1; ++j)
        file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      file_steer << i << " " << steer_target << " " << steer_output << "\n";
    }

    // ===================== THROTTLE / BRAKE =====================
    pid_throttle.UpdateDeltaTime(new_delta_time);

    double error_throttle = v_points[i0] - velocity;
    pid_throttle.UpdateError(error_throttle);
    double t_resp = pid_throttle.TotalError();

    double throttle_output = (t_resp > 0.0) ? t_resp : 0.0;
    double brake_output    = (t_resp > 0.0) ? 0.0   : -t_resp;

    if (file_throttle.good()) {
      file_throttle.seekg(std::ios::beg);
      for (int j = 0; j < i - 1; ++j)
        file_throttle.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      file_throttle << i << " " << error_throttle
                    << " " << brake_output
                    << " " << throttle_output << "\n";
    }

    // -------- send --------
    json out;
    out["brake"]    = brake_output;
    out["throttle"] = throttle_output;
    out["steer"]    = steer_output;

    out["trajectory_x"] = x_points;
    out["trajectory_y"] = y_points;
    out["trajectory_v"] = v_points;
    out["spirals_x"]    = spirals_x;
    out["spirals_y"]    = spirals_y;
    out["spirals_v"]    = spirals_v;
    out["spiral_idx"]   = best_spirals;
    out["active_maneuver"] = behavior_planner.get_active_maneuver();

    // for high update rate use 19; for slow use 4
    out["update_point_thresh"] = 16;

    auto msgOut = out.dump();
    i += 1;

    file_steer.close();
    file_throttle.close();

    ws.send(msgOut.data(), msgOut.length(), uWS::OpCode::TEXT);
  });

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    cout << "Connected!!!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    cout << "Disconnected" << endl;
  });

  int port = 4567;
  if (h.listen("0.0.0.0", port)) {
    cout << "Listening to port " << port << endl;
    h.run();
  } else {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }
}
