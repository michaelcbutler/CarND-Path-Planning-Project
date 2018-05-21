#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "road.h"
#include "vehicle.h"


using namespace std;

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
	Road r("../data/highway_map.csv");
	r.lane_width = 4.0; // meters

	Vehicle v;
	v.ref_vel = 0.0; // start from standstill
	v.target_vel = 49.5*0.44704; // mph converted to m/s

  h.onMessage([&r, &v](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      string s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          v.x = j[1]["x"];
          v.y = j[1]["y"];
          v.s = j[1]["s"];
          v.d = j[1]["d"];
          v.yaw = j[1]["yaw"];
          v.speed = j[1]["speed"];

					// Previous path data given to the Planner
					auto prev_path_x = j[1]["previous_path_x"];
					auto prev_path_y = j[1]["previous_path_y"];
					int prev_path_length = prev_path_x.size();
					v.previous_path_x.resize(prev_path_length);
					v.previous_path_y.resize(prev_path_length);
					for (int i = 0; i < prev_path_length; ++i) {
						v.previous_path_x[i] = prev_path_x[i];
						v.previous_path_y[i] = prev_path_y[i];
					}

          // Previous path's end s and d values 
          v.end_path_s = j[1]["end_path_s"];
          v.end_path_d = j[1]["end_path_d"];

					v.update_ref_state();

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];
					r.sensor_fusion.resize(sensor_fusion.size());
					for (int i = 0; i < sensor_fusion.size(); ++i) {
						r.sensor_fusion[i].resize(sensor_fusion[i].size());
						for (int j = 0; j < sensor_fusion[i].size(); ++j) {
							r.sensor_fusion[i][j] = sensor_fusion[i][j];
						}
					}

					v.compute_path(r);

					// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
					json msgJson;
					msgJson["next_x"] = v.next_x_vals;
					msgJson["next_y"] = v.next_y_vals;

					std::string msg = "42[\"control\","+ msgJson.dump()+"]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
