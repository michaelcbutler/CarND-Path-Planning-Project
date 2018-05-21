#ifndef ROAD_H
#define ROAD_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <math.h>


class Road {
  public:
    Road(std::string file_name);

    std::vector<double> getXY(double s, double d);

    std::vector<std::vector<double>> sensor_fusion;
    double lane_width;

  private:
    int ClosestWaypoint(double x, double y);
    int NextWaypoint(double x, double y, double theta);
    std::vector<double> getFrenet(double x, double y, double theta);

    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;

};


#endif