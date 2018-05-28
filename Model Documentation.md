## Path Planning Project: Model Documentation

### Code Organization
The code is based on the framework provided with extensions based on the walk-through. I have refactored the code, adding classes for `Road` and `Vehicle` and splitting the code into the files:
* `main.cpp`
* `vehicle.cpp`
* `vehicle.h`
* `road.cpp`
* `road.h`

The `Road` class encapsulates the framework methods:
* `vector<double> getXY(double s, double d) const;`
* `int ClosestWaypoint(double x, double y);`
* `int NextWaypoint(double x, double y, double theta)'`
* `vector<double> getFrenet(doiuble x, double y, double theta);`

The `Road` object is initialized with the waypoint map data and maintains updated `sensor_fusion` data.

### Event Loop (`main.cpp`)
The `Road` object and `Vehicle` objects are created and initialized:
```
Road r("../data/highway_map.csv");
r.lane_width = 4.0; // meters
r.lane_count = 3;

Vehicle v;
v.max_vel = 49.5*0.44704; // mph converted to m/s
v.delta_t = 0.02; // update rate (sec)
v.target_d = 0.5*(r.lane_count*r.lane_width) + 0.1;
```
and passed to the message handler as lambda parameters. The framework code for JSON parsing updates the `Vehicle` state, the reference state is updated (using the approach of the walk-through):
```
v.update_ref_state();
```
before updating the target state and finally generating path points for the next iteration:
```
v.update_target_state(r);
v.generate_path_points(r);
```

### Updating Reference State (`Vehicle::update_ref_state`)
As outlined in the walk-through video, the `Vehicle` reference state data (`ref_x`, `ref_y`, `ref_psi`, `ref_s`) describes the car's state at the end of the previously computed path. If the path does not yet exist (or has been fully consumed), the current state (`x`, `y`, `s`, and `yaw`) are used to approximate the reference state.

### Updating Target State (`Vehicle::update_target_state`)
The output of `update_target_state` is `target_d` and `target_vel`. These two parameters are sufficient to extend the current path.

The car has two states: maintaining current lane or changing lanes. The boolean member variable `changing_lanes` indicates the current state. The logic for updating target state:
* Check if lane change in progress has completed. If lane change still in progress, continue changing lanes.
* Otherwise:
  - Compute cost of lane change to the left
  - Compute cost of lane change to the right
  - Compute cost of maintaining current lane
  - Choose the lowest cost option (and update `target_vel` if necessary)
```
void Vehicle::update_target_state(const Road &r) {

  // update lane change state
  if (changing_lanes && lane_change_complete()) {
    //std::cout << "lane changed: d = " << d << std::endl;
    changing_lanes = false;
  }

  if (changing_lanes) {
    // adjust target_vel?
    target_vel = std::min(max_vel, target_vel + 0.1);
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
    //std::cout << "changing lanes: d = " << d << ", target_d = " << target_d << std::endl;
  }
```

#### Cost of maintaining current lane (`Vehicle::keep_lane_cost`)
The cost of maintaining the current lane is:
* 0 if unimpeded by traffic ahead and in center lane
* 150 if unimpeded by traffic ahead and not in center lane
* 200 if impeded by traffic ahead

The center lane is favored to provide more passing options. `target_vel` is adjusted based on current speed and traffic. If the car must maintain lane position behind traffic, then speed is reduced until it matches traffic ahead.

#### Cost of changing lanes (`Vehicle::lane_change_cost`)
The cost of changing lanes is:
* 1000 if no such lane exists (e.g., right lane change from right-most lane)
* 1000 if no space to merge (another car occupies the lane)
* 100 if space to merge exists

### Path Generation (`Vehicle::generate_path_points`)
Using `target_d` and `target_vel` from `update_target_state`, the car's path is extended using the method outlined in the walk-though:
* Using `ref_d`, `ref_s`, and `target_d`, define three equally-spaced waypoints beyond the reference point
* Transform the waypoints into the car's coordinate system
* Generate a spline defined by the predecessor point(`pred_x`, `pred_y`), reference point (`ref_x`, `ref_y`) and the transformed waypoints
* Divide the spline into intervals that correspond to `delta_t` increments based on `target_vel`
* Sample the spline at these intervals for the segment between the reference point and the first waypoint
* Construct the path for the next iteration from the remaining points of the previous path augmented with the spline points as transformed into global coordinates

## Performance Improvements
The current code seems to perform adequately in the simulation. However, possible peformance improvements include:
* Choosing optimal lane change from center lane. Currently the code prefers the left lane if it is available for passing. While passing on the left is generally preferred, sometimes passing on the right is the more efficient alternative.
* Better speed maintenance logic. Currently, speed is increased by 0.1 m/s until the speed limit is reached. This gives a 5 m/s^2 acceleration - below the maximum threshold of 10 specified for the project. Speed is reduced to match traffic when forward progress is blocked.