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

  // The simulator updates the car speed at relatively long intervals.
  // Therefore, to ensure smooth transitions in the path, a global variable
  // for velocity is kept and updated on each iteration.
  double vel = 0;

  h.onMessage([&vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          int path_size = previous_path_x.size();

            // Preventing collitions.
            if (path_size > 0) {
              car_s = end_path_s;
            }
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Pivot points for generating a smooth trajectory
          vector<double> spline_points_x;
          vector<double> spline_points_y;

          /**
           * Define a path made up of (x,y) points that the car will visit
           * sequentially every .02 seconds
           */

          // Points of reference from car coordinates to map coordinates
          double ref_x;
          double ref_y;
          double ref_yaw;

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
            ref_yaw = atan2(ref_y-pos_y2,ref_x-pos_x2);

            spline_points_x.push_back(pos_x2);
            spline_points_x.push_back(ref_x);
            spline_points_y.push_back(pos_y2);
            spline_points_y.push_back(ref_y);
          }


          // std::cout << "Ref: [" << ref_x << "," << ref_y << "]" << std::endl;

          /**
           * TODO:
           * Run the trajectory generation for (at most) three trajectories, each corresponding to a valid single lane change:
           * 1. Keep Lane
           * 2. Lane Change Left
           * 3. Lane Change Right
           * 
           * Calculate the cost for excuting each trajectory and select the one with the lowest cost.
           * Criteria:
           * 1. Collision (Penalizes trajectories which collide with vehicles)
           * 2. Buffer (Pelanizes trajectories which are closer to other vehicles)
           * 3. Efficiency (reward high average speeds)
           */

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


          // std::cout << "Converting To Local (Car) Coordinates" << std::endl;

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

          // std::cout << "Generating Spline" << std::endl;

          tk::spline s;
          s.set_points(spline_points_x, spline_points_y);

          // std::cout << "Spline Generated" << std::endl;

          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          // std::cout << "Previous Values Set" << std::endl;



          double target_x = 30.0; // some target in the future
          double target_y = s(target_x); // some target in the future
          double target_dist = distance(target_x, target_y, 0.0, 0.0);

          // std::cout << "Target Distance: " << target_dist << std::endl;

          // Velocity Control Logic
          double speed_ms = car_speed * 0.44704; // convert from mph to ms-1
          const double MAX_ACCEL = 10;
          const double MAX_VEL = 49.5 * 0.44704;
          const double ACCEL_TIME = 0.01; // Acceleration is measured in 0.2s invervals by the simulator, but the messages update every 0.02s (0.02/0.2 = 0.1 for proper scaling)
          const double VEL_BUFFER = MAX_ACCEL*ACCEL_TIME; // Velocity buffer to ensure car stays within limits with controller error
          // double vel;

          
          // std::cout << "The next valid states are: " << std::endl;
          // vector<State> lane_changes = validStates(car_d);
          // for(auto &s: lane_changes){
          //   string print_statement = "";
          //   if(s == State::KL){
          //     print_statement = "Keep Lane";
          //   }
          //   else if(s == State::LCR){
          //     print_statement = "Lane Change Right";
          //   }
          //   else if(s == State::LCL){
          //     print_statement = "Lane Change Left";
          //   }

          //   std::cout << print_statement << std::endl;
          // }

          // Velocity Limiter for Lane Keeping
          double target_vel = MAX_VEL;
          for(auto &s: sensor_fusion){
            if(s[6] >= 4 && s[6] <= 8 && s[5] > car_s){
                double x_map = s[1];
                double y_map = s[2];
                // std::cout << "Following Vehicle Coordinates: [" << x_map << "," << y_map << "]" << std::endl;

                double veh_dist = distance(x_map, y_map, ref_x, ref_y);
                // std::cout << "Following Vehicle Distance: " << veh_dist << std::endl;
                if(veh_dist < target_dist){
                  double x_vel = s[3];
                  double y_vel = s[4];
                  double vel_mag = sqrt(x_vel*x_vel + y_vel*y_vel);
                  if(vel_mag < target_vel){
                    // std::cout << "Setting new target velocity " << target_vel << "-> " << vel_mag << std::endl;
                    target_vel = vel_mag;
                    // std::cout << "New target velocity set: " <<target_vel << std::endl;
                  }
                }
            } 
          }


          // std::cout << "Speed Diff: " << speed_diff << std::endl;


          /** 
           * Keeping track of the most recent x-point to allow variable velocity increments
           * at each path point. This is required especially when the car is starting from
           * idle, where path points may cluster and cause the car to jitter.
           * NOTE: this does not matter when velocity is constant, but is important to ensure 
           * smooth motion when velocity can change.
           */
          double x_ref = 0;
          // Path point generation
          for(int i = 0; i < 50-path_size; ++i){
            // Velocity based on max acceleration
            // unless speed limit reached
            double max_vel_buf = target_vel - VEL_BUFFER;
            // For speed differences which are smaller than the max 
            // incremental velocity change
            double speed_diff = fabs(target_vel-vel);
            if(speed_diff > VEL_BUFFER){
              speed_diff = VEL_BUFFER;
            }
            // Increase velocity if within limits
            if(vel <= max_vel_buf){
              vel += speed_diff;
            }
            // Decrease velocity if exceeded limits
            else if(vel > max_vel_buf){
              vel -= speed_diff;
            } 
            double N = target_dist/(0.02*vel);
            double point_x = x_ref + target_x/N;
            double point_y = s(point_x);

            x_ref = point_x;

            // Transforming the points back to map coordinates
            // Counter-clockwise rotation and translation
            double point_x_ = point_x*cos(ref_yaw) - point_y*sin(ref_yaw);
            double point_y_ = point_x*sin(ref_yaw) + point_y*cos(ref_yaw);

            // Translating back to map coordinates
            point_x_ += ref_x;
            point_y_ += ref_y;

            next_x_vals.push_back(point_x_);
            next_y_vals.push_back(point_y_);

          }

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