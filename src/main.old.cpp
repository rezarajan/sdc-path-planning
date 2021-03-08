#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
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

          // Pivot points for generating a smooth trajectory
          vector<double> spline_points_x;
          vector<double> spline_points_y;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // Points of reference from car coordinates to map coordinates
          double ref_x;
          double ref_y;
          double ref_yaw;
          int path_size = previous_path_x.size();

          if (path_size < 2) {
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);
            spline_points_x.push_back(ref_x);
            spline_points_y.push_back(ref_y);
          } else {
            ref_x = previous_path_x[path_size-1];
            ref_y = previous_path_y[path_size-1];

            double pos_x2 = previous_path_x[path_size-2];
            double pos_y2 = previous_path_y[path_size-2];
            ref_yaw = atan2(ref_x-pos_y2,ref_y-pos_x2);

            spline_points_x.push_back(pos_x2);
            spline_points_x.push_back(ref_x);
            spline_points_y.push_back(pos_y2);
            spline_points_y.push_back(ref_y);
          }


          std::cout << "Ref: [" << ref_x << "," << ref_y << "]" << std::endl;

          int lane = 1; // TODO: Change this
          // Setting up target points in the future for the trajectory
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          spline_points_x.push_back(next_wp0[0]);
          spline_points_x.push_back(next_wp1[0]);
          spline_points_x.push_back(next_wp2[0]);

          spline_points_y.push_back(next_wp0[1]);
          spline_points_y.push_back(next_wp1[1]);
          spline_points_y.push_back(next_wp2[1]);


          std::cout << "Converting To Local (Car) Coordinates" << std::endl;

        /**
         * NOTE: The waypoints are converted to local car coordinates first
         * because keeping global coordinates may result in a spline which
         * spans large values, and this can lead to adverse effects with point
         * generation.
         */
        // Rotating the points clockwise, with the car as the origin
          for(int i = 0; i < spline_points_x.size(); ++i){
            double x = spline_points_x[i];
            double y = spline_points_y[i];
            x -= ref_x;
            y -= ref_y;

            double x_ = x * cos(ref_yaw) + y * sin(ref_yaw);
            double y_ = -x * sin(ref_yaw) + y * cos(ref_yaw);

            spline_points_x[i] = x_;
            spline_points_y[i] = y_;
          }

          std::cout << "Generating Spline" << std::endl;

          tk::spline s;
          s.set_points(spline_points_x, spline_points_y);

          std::cout << "Spline Generated" << std::endl;

          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          std::cout << "Previous Values Set" << std::endl;



          double target_x = 30.0; // some target in the future
          double target_y = s(target_x); // some target in the future
          double target_dist = distance(target_x, target_y, 0.0, 0.0);

          std::cout << "Target Set" << std::endl;

          // Velocity Control Logic
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

          // double dist_inc = vel * 0.02;
          // double target_inc = dist_inc/target_dist;
          // std::cout << "Target Distance: " << target_dist << "m" << std::endl;
          // std::cout << "Velocity: " << vel << "ms-1" << std::endl;
          int counter = 1;
          double ref_vel = speed_ms;
          for(int i = 0; i < 50-path_size; ++i){
            double speed_diff = 0.0;
            if(ref_vel < MAX_VEL){
              ref_vel += speed_diff;
            }
            if( ref_vel > MAX_VEL ) {
              ref_vel = MAX_VEL;
            } else if ( ref_vel < MAX_ACCEL ) {
              ref_vel = MAX_ACCEL;
            }
            double N = target_dist/(0.02*ref_vel);
            double point_x = 0.1*counter;
            double point_y = s(point_x);

            // Transforming the points back to map coordinates
            // Counter-clockwise rotation and translation
            double point_x_ = point_x*cos(ref_yaw) - point_y*sin(ref_yaw);
            double point_y_ = point_x*sin(ref_yaw) + point_y*cos(ref_yaw);
            point_x_ += ref_x;
            point_y_ += ref_y;

            next_x_vals.push_back(point_x_);
            next_y_vals.push_back(point_y_);
            ++counter;
          }

  



          // Circular Motion
          // for (int i = 0; i < 50-path_size; ++i) {    
          //   next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
          //   next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
          //   pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
          //   pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
          // }


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