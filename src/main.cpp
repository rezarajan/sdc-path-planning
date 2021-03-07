#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // Testing straight-line path
          double speed_ms = car_speed * 0.44704; // convert from mph to ms-1
          const double MAX_ACCEL = 10;
          const double MAX_VEL = 50 * 0.44704;
          const double VEL_BUFFER = 0.5; // Velocity buffer to ensure car stays within limits with controller error
          double vel;
          // Velocity based on max acceleration
          // unless speed limit reached
          if(speed_ms < MAX_VEL){
            vel = MAX_ACCEL*0.02 + speed_ms;
          }
          else{
            // vel = MAX_VEL-VEL_BUFFER;
            vel = speed_ms - MAX_ACCEL*0.02;
          } 
          double dist_inc = vel * 0.02;

          // Normal Acceleration Check
          double rad_curv = pow(vel,2)/MAX_ACCEL;

          // double new_angle = dist_inc/rad_curv;

          // Using waypoint data to calculate the maximum time for trajectory
          // based on Frenet s distance
          // int waypoint = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
          // double time = map_waypoints_s[waypoint]/vel;

          /**
           * TODO: Generate path using time
           */

          // std::cout << time << std::endl;


          // double dist_inc = 0.5;
          // std::cout << rad_curv << std::endl;

          // dist_inc = vel * 0.02;

          double pos_x;
          double pos_y;
          double pos_s;
          double angle;
          int path_size = previous_path_x.size();

          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          if (path_size == 0) {
            pos_x = car_x;
            pos_y = car_y;
            pos_s = car_s;
            // angle = deg2rad(car_yaw);
          } else {
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];
            double pos_x2 = previous_path_x[path_size-2];
            double pos_y2 = previous_path_y[path_size-2];
            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);

            vector<double> frenet = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
            pos_s = frenet.data()[0];
          }

          // Straight Line
          for (int i = 0; i < 50-path_size; ++i) {
            double dist_inc_ = 0.25;
            double ds = pos_s + (dist_inc_*i);
            vector<double> cartesian = getXY(ds, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);


            //Rescale output to obey acceleration and velocity constraints
            double cartesian_dist = distance(cartesian.data()[0], cartesian.data()[1], car_x, car_y);
            if(cartesian_dist > dist_inc){
              std::cout << "Incremental distance magnitude exceeded: cartesian: [" << cartesian_dist << "]m | incremental: [" << dist_inc << "]m" << std::endl;
            }

            // next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
            // next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
            next_x_vals.push_back(cartesian.data()[0]);
            next_y_vals.push_back(cartesian.data()[1]);
          }

          // double pos_x;
          // double pos_y;
          // double angle;
          // int path_size = previous_path_x.size();

          // for (int i = 0; i < path_size; ++i) {
          //   next_x_vals.push_back(previous_path_x[i]);
          //   next_y_vals.push_back(previous_path_y[i]);
          // }

          // if (path_size == 0) {
          //   pos_x = car_x;
          //   pos_y = car_y;
          //   angle = deg2rad(car_yaw);
          // } else {
          //   pos_x = previous_path_x[path_size-1];
          //   pos_y = previous_path_y[path_size-1];

          //   double pos_x2 = previous_path_x[path_size-2];
          //   double pos_y2 = previous_path_y[path_size-2];
          //   angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
          // }

          // // Circular Motion
          // for (int i = 0; i < 50-path_size; ++i) {   
          //   pos_x += (dist_inc)*cos(angle+(i+1)*new_angle);
          //   pos_y += (dist_inc)*sin(angle+(i+1)*new_angle);
          //   next_x_vals.push_back(pos_x);
          //   next_y_vals.push_back(pos_y);
          //   // next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
          //   // next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
          //   // pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
          //   // pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
          // }

          // std::cout << map_waypoints_x.size() << std::endl;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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