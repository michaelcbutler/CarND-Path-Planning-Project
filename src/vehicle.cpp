#include <math.h>
#include <iostream>
#include "vehicle.h"
#include "spline.h"

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

void Vehicle::update_ref_state() {

  prev_path_length = previous_path_x.size();

  if (prev_path_length > 1) {
    // use last two points in previous path
    ref_x = previous_path_x.back();
    ref_y = previous_path_y.back();
    int pred_index = prev_path_length - 2;
    pred_x = previous_path_x[pred_index];
    pred_y = previous_path_y[pred_index];
    ref_psi = atan2(ref_y - pred_y, ref_x - pred_x);
    ref_s = end_path_s;
  }
  else {
    // use current position and yaw to generate points
    ref_psi = deg2rad(yaw);
    ref_x = x;
    ref_y = y;
    pred_x = x - cos(ref_psi);
    pred_y = y - sin(ref_psi);
    ref_s = s;
  }
 
}

void Vehicle::generate_path_points(const Road &r) {

  // some hyperparameters
  const double D = 30.0; // waypoint spacing (meters) // TODO: too large?

  // define 3 waypoints beyond previous path
  std::vector<double> wp0 = r.getXY(ref_s + D,   target_d);
  std::vector<double> wp1 = r.getXY(ref_s + 2*D, target_d);
  std::vector<double> wp2 = r.getXY(ref_s + 3*D, target_d);

  // path definition in world coordinates
  std::vector<double> w_x = {pred_x, ref_x, wp0[0], wp1[0], wp2[0]}; // world x
  std::vector<double> w_y = {pred_y, ref_y, wp0[1], wp1[1], wp2[1]}; // world y

  // transform into vehicle coordinate system with origin at reference point
  std::vector<double> v_x; // vehicle x
  std::vector<double> v_y; // vehicle y
  for (int i = 0; i < w_x.size(); ++i) {
    double delta_x = w_x[i] - ref_x;
    double delta_y = w_y[i] - ref_y;
    double delta_psi = -ref_psi;

    double x = delta_x*cos(delta_psi) - delta_y*sin(delta_psi);
    double y = delta_x*sin(delta_psi) + delta_y*cos(delta_psi);

    v_x.push_back(x);
    v_y.push_back(y);
  }

  // create a spline from path points
  tk::spline s;
  s.set_points(v_x, v_y);

  // path for next iteration initialized to previous path
  next_x_vals.resize(prev_path_length);
  next_y_vals.resize(prev_path_length);
  for (int i = 0; i < prev_path_length; ++i) {
    next_x_vals[i] = previous_path_x[i];
    next_y_vals[i] = previous_path_y[i];
  }

  double target_x = D;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y); // approx.
  double N = target_dist/(delta_t*target_vel);
  double delta_x = target_x/N;
  double x_v = 0.0;
  double y_v; 

  const int MAX_PATH_LENGTH = 50;
  for (int i = prev_path_length; i < MAX_PATH_LENGTH; ++i) {
    x_v += delta_x;
    y_v = s(x_v);

    // transform back into world coordinates
    double x_w = ref_x + (x_v*cos(ref_psi) - y_v*sin(ref_psi));
    double y_w = ref_y + (x_v*sin(ref_psi) + y_v*cos(ref_psi));

    next_x_vals.push_back(x_w);
    next_y_vals.push_back(y_w);
  }

}

bool lane_overlap(double d, double target_d, double lane_width)	{
	double min_d = target_d - 0.5*lane_width;
	double max_d = target_d + 0.5*lane_width;
	return (d > min_d) && (d < max_d);
}

bool too_close(double s, double ref_s, double front_buffer = 30., double rear_buffer = 0.) {
	if ((s > ref_s) && ((s - ref_s) < front_buffer))
    return true; // too close in front
  else 
    return ((ref_s > s) && (ref_s - s) < rear_buffer);
}

bool Vehicle::car_close_ahead(const Road &r) {

  for (int i = 0; i < r.sensor_fusion.size(); ++i) {
		double d = r.sensor_fusion[i][6];
		if (!lane_overlap(d, target_d, r.lane_width))
			continue; // not in our lane
		double vx = r.sensor_fusion[i][3];
		double vy = r.sensor_fusion[i][4];
		double vel = sqrt(vx*vx + vy*vy);
		double s = r.sensor_fusion[i][5] + prev_path_length*delta_t*vel; // other car est position in future
		if (too_close(s, ref_s))
			return true;
	}
	return false;
 
}

bool Vehicle::lane_change_blocked(const Road &r, double proposed_d) {

  for (int i = 0; i < r.sensor_fusion.size(); ++i) {
		double d = r.sensor_fusion[i][6];
		if (!lane_overlap(d, proposed_d, r.lane_width))
			continue; // not in the lane we want
		double vx = r.sensor_fusion[i][3];
		double vy = r.sensor_fusion[i][4];
		double vel = sqrt(vx*vx + vy*vy);
		double s = r.sensor_fusion[i][5] + prev_path_length*delta_t*vel; // other car est position in future
		if (too_close(s, ref_s, 30.0, 10.0))
			return true;
	}
	return false;
}

int Vehicle::lane_change_cost(const Road &r, double proposed_d) {
  // driving off the road?
  if (proposed_d < 0 || proposed_d > r.lane_count*r.lane_width)
    return 1000;

  if (lane_change_blocked(r, proposed_d))
    return 1000;

  return 100;
}

int Vehicle::keep_lane_cost(const Road &r) {
  // if car impeding progress, then 200
  // otherwise zero
  if (car_close_ahead(r)) {
    target_vel -= 0.05;
    return 200;
  }
  else if (target_vel < max_vel)
    target_vel += 0.1;

  return 0;
}

bool Vehicle::lane_change_complete() {
  return (abs(target_d - d) < 0.1); // close enough to target_d?
}

void Vehicle::update_target_state(const Road &r) {

  // update lane change state
  changing_lanes = changing_lanes && !lane_change_complete();

  if (changing_lanes) {
    // adjust target_vel?
    // target_vel = min(max_vel, target_vel + 0.1);
    return;  // finish lane change in progress
  }

  // find least costly lane change option
  int lane_change_d = target_d - r.lane_width; // left lane change
  int min_change_cost = lane_change_cost(r, lane_change_d);

  int right_d = target_d + r.lane_width;
  int right_cost = lane_change_cost(r, right_d);
  if (right_cost < min_change_cost) {
    min_change_cost = right_cost;
    lane_change_d = right_d;
  }

  // compare lane change cost with keep lane
  changing_lanes = keep_lane_cost(r) > min_change_cost;

  if (changing_lanes) {
    target_d = lane_change_d;
  }

}

