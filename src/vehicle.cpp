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

void Vehicle::compute_path(Road & r) {

  // some hyperparameters
  const double delta_t = 0.02; // seconds per iteration
  const double D = 30.0; // waypoint spacing (meters) // TODO: too large?
  const int target_lane = 1; // middle lane, RHS

  double target_d = (target_lane + 0.5)*r.lane_width;

  std::cout << "prev_path_length = " << prev_path_length << std::endl;
  // TODO: resize previous_path if too long?

  // slow down for some car too close ahead
  if (car_close_ahead(r.sensor_fusion, target_d, delta_t, r.lane_width)) {
    //ref_vel -= 0.1;
    target_d -= r.lane_width;
  }
  else if (ref_vel < target_vel) {
    ref_vel += 0.1;
  }

  std::vector<double> wp0 = r.getXY(ref_s + D,   target_d);
  std::vector<double> wp1 = r.getXY(ref_s + 2*D, target_d);
  std::vector<double> wp2 = r.getXY(ref_s + 3*D, target_d);

  // path definition in world coordinates
  std::vector<double> w_x = {pred_x, ref_x, wp0[0], wp1[0], wp2[0]};
  std::vector<double> w_y = {pred_y, ref_y, wp0[1], wp1[1], wp2[1]};

  // transform into vehicle coordinate system with origin at reference point
  std::vector<double> v_x;
  std::vector<double> v_y;
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
  double target_dist = sqrt(target_x*target_x + target_y*target_y);
  double N = target_dist/(delta_t*ref_vel);
  double delta_x = target_x/N;
  double x_v = 0.0;
  double y_v; 

  //cout << "N = " << N << endl;

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

bool lane_overlap(double d, double target_d, double LANE_WIDTH)
	{
	double min_d = target_d - 0.5*LANE_WIDTH;
	double max_d = target_d + 0.5*LANE_WIDTH;
	return (d > min_d) && (d < max_d);
		}

bool too_close(double s, double ref_s)
{
	const double MIN_DIST = 30.0;
	return (s > ref_s) && ((s - ref_s) < MIN_DIST);
  }

bool Vehicle::car_close_ahead(const std::vector<std::vector<double>>& sensor_fusion, double target_d, double delta_t, double LANE_WIDTH) {

  for (int i = 0; i < sensor_fusion.size(); ++i) {
		double d = sensor_fusion[i][6];
		if (!lane_overlap(d, target_d, LANE_WIDTH))
			continue;
		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double vel = sqrt(vx*vx + vy*vy);
		double s = sensor_fusion[i][5] + prev_path_length*delta_t*vel; // other car est position in future
		if (too_close(s, ref_s))
			return true;
	}
	return false;
 
}
