#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include "road.h"

class Vehicle {
public:
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;

  double ref_vel;
  double target_vel;

  double ref_x;
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

  Vehicle() {}

  void update_ref_state();
  void compute_path(Road& r);

private:

  bool car_close_ahead(const std::vector<std::vector<double>>& sensor_fusion, double target_d, double delta_t, double LANE_WIDTH);

};

#endif