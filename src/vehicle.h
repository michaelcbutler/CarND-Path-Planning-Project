#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include "road.h"

class Vehicle {
public:
  double delta_t; // time interval for path point generation
  double max_vel; // speed limit

  bool changing_lanes;

  double x; // current state
  double y;
  double s;
  double d;
  double yaw;
  double speed;

  double target_vel;
  double target_d;

  double ref_x;   // position at end of previous path
  double ref_y;
  double ref_psi;	// car yaw angle 
  double ref_s;   // s coordinate at reference point

  double pred_x;  // position before reference point
  double pred_y;

  std::vector<double> previous_path_x;
  std::vector<double> previous_path_y;
  int prev_path_length;

  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  double end_path_s;
  double end_path_d;

  Vehicle() : changing_lanes(false) {}

  void update_ref_state();

  void update_target_state(const Road &r);
  void generate_path_points(const Road &r);

private:

  bool no_space_to_merge(double est_s, double front_buffer = 30., double rear_buffer = 0.);

  bool car_close_ahead(const Road &r, double &match_vel);
  bool lane_change_complete();

  int keep_lane_cost(const Road &r);
  int lane_change_cost(const Road &r, double proposed_d);

};

#endif