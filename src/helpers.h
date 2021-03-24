#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"
#include "Eigen-3.3/Eigen/Dense"

// for convenience
using std::string;
using std::vector;
using Eigen::MatrixXd;

enum class State {KL, LCR, LCL};

const int PATH_LENGTH = 55;
// Velocity Control Constants
const double MAX_ACCEL = 10;
const double MAX_VEL = 49.5 * 0.44704;
const double ACCEL_TIME = 0.01; // Acceleration is measured in 0.2s invervals by the simulator, but the messages update every 0.02s (0.02/0.2 = 0.1 for proper scaling)
const double VEL_BUFFER = MAX_ACCEL*ACCEL_TIME; // Velocity buffer to ensure car stays within limits with controller error
const double TIMESTEP = 0.02; // Simulator update rate
const double BRAKING_DIST = 55;
const double MAX_COLLISION_RADIUS = 25;
const double MAX_COLLISION_DIST_S = 20;
const double MIN_BRAKING_VEL = 5.0;

bool transitioning = false;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}


/**
 * This function returns the next valid lane change states for the ego vehicle
 * @param lane - internally tracked lane number
 * @param ego_lane - lane number of the ego vehicle
 */ 
vector<State> validStates(const int &lane, const int &ego_lane, const bool &collision, const bool &braking_only){
  // Find the next states for the vehicle
  vector<State> valid_states;
  // If vehicle has already transitioned to the tracked target lane
  if(!braking_only && (collision || (lane == ego_lane))){
    switch(lane) {
      case(0):
        // If in leftmost lane, then Keep Lane or Lane Change Right
        valid_states = {State::KL, State::LCR};
        break;
      case(1):
        // If in center lane, then Keep Lane, Lane Change Right or Lane Change Left
        valid_states = {State::KL, State::LCR, State::LCL};
        break;
      case(2):
        // If in rightmost lane, then Keep Lane or Lane Change Left
        valid_states = {State::KL, State::LCL};
        break;
      default:
        // Default to just Keep Lane
        valid_states = {State::KL};
        break;
    }
    transitioning = false;
  }
  else {
    // Otherwise retain the trakced lane until transition is complete
    if(braking_only){
      std::cout << "Braking Only!" << std::endl;
      int lane_diff = lane - ego_lane;
      if(lane_diff == 0){
        valid_states = {State::KL};
      }
      else if(lane_diff == 1){
        valid_states = {State::LCL};
      }
      else if(lane_diff == -1){
        valid_states = {State::LCR};
      }
      transitioning = false;
    }
    else {
      valid_states = {State::KL};
      transitioning = true;
    }
  }

  return valid_states;
}


 /**
   * Find the closest vehicles both ahead and behind the car, in the target lane. Returns the target velocity
   * for path generation.
   * @param vehicle_telemetry ego vehicle map x, map y, heading, Frenet s, Frenet d
   * @param sensor_fusion sensor fusion data {car id, map x, map y, velocity x, velocity y, Frenet s, Frenet d}
   * @param ref_x reference point x on path (usually at the end point of the previous path if there is one)
   * @param ref_y reference point y on path (usually at the end point of the previous path if there is one)
   * @param target vector of Frenet {s, d} coordinates, where s is relative to the reference point
  */ 
double getTargetVelocity(const vector<double> &vehicle_telemetry, const vector<vector<double>> &sensor_fusion,
                        const double &ref_x, const double &ref_y, const vector<double> &target){

    double car_s = vehicle_telemetry[3];

    // Find all vehicles in the target lane
    vector<vector<double>> target_vehicles;
    int target_lane = (int)floor((target[1] - 1)/4);
    if(target_lane < 0){
      target_lane = 0;
    }

    // Get the current ego vehicle lane (which may be different than the target internally tracked lane)
    int ego_lane = (int)floor((vehicle_telemetry[4] - 1)/4);
    if(ego_lane < 0){
      ego_lane = 0;
    }

    for(const auto &s: sensor_fusion){
      int s_lane = (int)floor((s[6] - 1)/4);
      if(s_lane < 0){
        s_lane = 0;
      }

      if(s_lane == target_lane){
        double x_map = s[1];
        double y_map = s[2];
        double x_vel = s[3];
        double y_vel = s[4];
        double dist = distance(x_map, y_map, ref_x, ref_y);
        double vel = sqrt(x_vel*x_vel + y_vel*y_vel);
        
        target_vehicles.push_back({dist, s[5], vel});
      }
    }

    // Sort by smallest distance
    if(target_vehicles.size() > 1){
        std::sort(target_vehicles.begin(), target_vehicles.end(), 
        [](const vector<double> &v_a, const vector<double> & v_b) 
        { return v_a[0] < v_b[0]; });
    }

    // Find the two closest vehicles, each either ahead or behind the ego vehicle
    bool car_ahead = false;
    // bool car_behind = false;
    double target_velocity = MAX_VEL;
    // double car_ahead_vel;
    // double car_behind_vel;
    // double car_ahead_dist = target[0];
    // double car_behind_dist = target[0];
    for(int i = 0; i < target_vehicles.size(); ++i){
      // if(car_ahead && car_behind){
      //   break;
      // }
      if(target_vehicles[i][1] >= car_s){
      // if(!car_ahead && (target_vehicles[i][1] >= car_s)){
        // Set a minimum target velocity if there is a car ahead, and in range of target distance
        if((target_vehicles[i][2] <= MAX_VEL)){
          if(target_vehicles[i][0] < target[0]){
            // car_ahead_dist = target_vehicles[i][0];
            target_velocity = target_vehicles[i][2];
            // Do not reduce velocity unless entirely in target lane (prevents slowing down too much during lane change)
            // if(ego_lane == target_lane){
            if(vehicle_telemetry[4] == target[1]){
              // In the case where a car suddenly merges reduce the target velocity to widen the gap
              double target_dist = target_vehicles[i][1] - car_s;
              // double target_dist = target_vehicles[i][0];
              if(target_dist < BRAKING_DIST){
                target_velocity = (target_vehicles[i][2]-target_velocity) + 2*(0.5*MAX_ACCEL)*BRAKING_DIST;
                target_velocity = std::min(target_velocity, MAX_VEL);
                target_velocity = std::min(target_velocity, MIN_BRAKING_VEL);
              }
              // while(target_dist < BRAKING_DIST){
                // v^2 = u^2 + 2as
                // double target_velocity_ = target_velocity - VEL_BUFFER;
                // if(target_velocity_ > MIN_BRAKING_VEL){
                  // target_velocity = target_velocity_;
                  // target_dist += (target_vehicles[i][2]-target_velocity)*TIMESTEP;
                // }
                // else{
                //   break;
                // }
              // }
            }
          }
        }
        // car_ahead = true;
        break;
      }
    }
    //   if(!car_behind && (target_vehicles[i][1] < car_s)){
    //     if(target_vehicles[i][0] < target[0]){
    //       car_behind_dist = target_vehicles[i][0]; 
    //       car_behind_vel = target_vehicles[i][2];
    //       car_behind = true;
    //     }
    //   }
    // }

    // if(car_behind && (car_behind_vel > target_velocity)){
    //   if(car_behind_dist < car_ahead_dist){
    //     target_velocity = std::min(car_behind_vel, MAX_VEL);
    //   }
    // }

    return target_velocity;
}

  /**
   * Generates a spline given the start and end positions in Frenet coordinates
   * @param start starting Frenet coordinates as a vector of {s, d} relative to the ego vehicle
   * @param end end Frenet coordinates as a vector of {s, d} relative to the ego vehicle
   * @param vel internally tracked velocity of the ego vehicle 
   * @param target_vel maximum velocity possible for the target lane 
   * @param previous_path previous path points to include in trajectory generation in map {x,y} coordinates
   * @param end_path the Frenet coordinates of the end of the previous path {s,d}
   * @param vehicle_telemetry ego vehicle map x, map y, heading, Frenet s, Frenet d
   * @param sensor_fusion sensor fusion data {car id, map x, map y, velocity x, velocity y, Frenet s, Frenet d}
   * @param map_waypoints vector of map waypoint {Frenet s, x, y} coordinates
  */ 
vector<vector<double>> generateTrajectory(const vector<double> &start, 
                                          const vector<double> &end, 
                                          double &vel, double &target_vel,
                                          const vector<double> &previous_path_x,
                                          const vector<double> &previous_path_y,
                                          const vector<vector<double>> &sensor_fusion, 
                                          const vector<double> &end_path,
                                          const vector<double> &vehicle_telemetry, 
                                          const vector<double> &map_waypoints_s,
                                          const vector<double> &map_waypoints_x,
                                          const vector<double> &map_waypoints_y){
  
    int path_size = previous_path_x.size();
    double car_s = vehicle_telemetry[3];

    // Continue path generation from end of previous path
    if (path_size > 0) {
      car_s = end_path[0];
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

    // Points of reference in map coordinates
    double ref_x;
    double ref_y;
    double ref_yaw;

    if (path_size < 2) {
      ref_x = vehicle_telemetry[0];
      ref_y = vehicle_telemetry[1];
      ref_yaw = deg2rad(vehicle_telemetry[2]);
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

    // Define spline midpoints
    double mid_s = 0.5*(start[0]+end[0]);
    double mid_d = 0.5*(start[1]+end[1]);
    // Setting up target points in the future for the trajectory
    vector<double> next_wp0 = getXY(car_s + start[0], start[1], map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + mid_s   , mid_d   , map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + end[0]  , end[1]  , map_waypoints_s, map_waypoints_x, map_waypoints_y);

    spline_points_x.push_back(next_wp0[0]);
    spline_points_x.push_back(next_wp1[0]);
    spline_points_x.push_back(next_wp2[0]);

    spline_points_y.push_back(next_wp0[1]);
    spline_points_y.push_back(next_wp1[1]);
    spline_points_y.push_back(next_wp2[1]);


  /**
   * NOTE: The waypoints are converted to local car coordinates first
   * because keeping global coordinates may result in a spline which
   * spans large values, and this can lead to adverse effects with point
   * generation.
   */
  // Rotating the points clockwise, setting the ego vehicle as the origin
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

    // Generate spline using the target points
    tk::spline s;
    s.set_points(spline_points_x, spline_points_y);


    for (int i = 0; i < path_size; ++i) {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }


    double target_x = 30.0; // some target in the future
    double target_y = s(target_x); // some target in the future
    double target_dist = distance(target_x, target_y, 0.0, 0.0);

    // Velocity Limiter
    vector<double> target = {target_dist, end[1]};
    target_vel = getTargetVelocity(vehicle_telemetry, sensor_fusion, ref_x, ref_y, target);


    /** 
     * Keeping track of the most recent x-point to allow variable velocity increments
     * at each path point. This is required especially when the car is starting from
     * idle, where path points may cluster and cause the car to jitter.
     * NOTE: this does not matter when velocity is constant, but is important to ensure 
     * smooth motion when velocity can change.
     */
    double x_ref = 0;
    // Path point generation
    for(int i = 0; i < PATH_LENGTH-path_size; ++i){
      // Velocity based on max acceleration
      // unless target speed reached
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
      double N = target_dist/(TIMESTEP*vel);
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
    vector<vector<double>> trajectory = {next_x_vals, next_y_vals};
    return trajectory;
}

/**
 * Cost functions --------------------------------------------------------------------------
 */ 

/**
 * Binary cost function which penalizes collisions
 * @param trajectory  vector of pair of {x,y} trajectory points visited every 0.02 seconds
 * @param sensor_fusion  surrounding vehicles id, map x, map y, vel x, vel y, Frenet s, Frenet d
 * @param vehicle_telemetry  ego vehicle map x, map y, heading, Frenet s, Frenet d
 * @param lane  target lane number
*/ 
double collisionCost(const vector<vector<double>> &trajectory, const vector<vector<double>> &sensor_fusion,
                    const vector<double> &vehicle_telemetry, const int &lane){
  double cost = 0;
  double trajectory_size = trajectory[0].size();

  vector<double> distances;
  
  // Get the current ego vehicle lane (which may be different than the target internally tracked lane)
  int ego_lane = (int)floor((vehicle_telemetry[4] - 1)/4);
  if(ego_lane < 0){
    ego_lane = 0;
  }
  // Check all surrounding vehicles
  for(const auto &s: sensor_fusion){
    // Across all timesteps along the trajectory
    double x_map = s[1];
    double y_map = s[2];
    double x_vel = s[3];
    double y_vel = s[4];
    double vel = sqrt(x_vel*x_vel + y_vel*y_vel);


    int s_lane = (int)floor((s[6] - 1)/4);
    if(s_lane < 0){
      s_lane = 0;
    }

    double vehicle_dist = fabs(s[5] - vehicle_telemetry[3]);
    // Collision on same lane is different than for merging lanes (braking allowance)
    bool same_lane_collision = (ego_lane == s_lane) && (vehicle_dist <= MAX_COLLISION_DIST_S);
    if(same_lane_collision){
      std::cout << "Same Lane Collision!" << std::endl;
      cost = 1.0;
      return cost;
    }

    if((s_lane == lane)){
      for(int t = 0; t < trajectory_size; ++t){
        x_map += TIMESTEP*x_vel;
        y_map += TIMESTEP*y_vel;
        // Check for the closest approach to the trajectory
        double dist = distance(x_map, y_map, trajectory[0][t], trajectory[1][t]);
        distances.push_back(dist);
        // And if there is a collision, return a cost of 1
        if(dist <= MAX_COLLISION_RADIUS){
          cost = 1.0;
          return cost;
        }
      }
    }
  }
  return cost;
}

/**
 * Cost function which penalizes speeds slower than the maximum velocity
 * @param target_velocity end target velocity for the trajectory in ms^-1
*/ 
double efficiencyCost(const double &target_velocity){
  double x = (MAX_VEL-target_velocity)/MAX_VEL;
  return 2.0 / (1 + exp(-x)) - 1.0;
}

/**
 * Cost function which penalizes changing lanes
 * @param current_lane internally tracked lane
 * @param target_lane target lane for the trajectory
*/ 
double laneChangeCost(const int &current_lane, const int &target_lane){
  return current_lane == target_lane ? 0.0 : 1.0;
}

/**
 * Cost function which penalizes lanes which are occupied by more cars (ahead of ego vehicle)
 * @param target_lane target lane for the trajectory
 * @param sensor_fusion  surrounding vehicles id, map x, map y, vel x, vel y, Frenet s, Frenet d
 * @param car_s Frenet s value for the ego vehicle
*/ 
double laneOccupancyCost(const int &target_lane, const vector<vector<double>> &sensor_fusion, const double &car_s){
  double counter = 0;
  for(const auto &s: sensor_fusion){
    int s_lane = (int)floor((s[6] - 1)/4);
    if(s_lane < 0){
      s_lane = 0;
    }
    // Vehicle is in the target lane and ahead of ego
    if((s_lane == target_lane) && (s[5] > car_s)){
      counter++;
    }
  }
  return counter;
}

/**
 * Cost function which penalizes deviations from the center lane
 * @param current_lane internally tracked lane
*/ 
double centerDeviationCost(const int &current_lane){
  return current_lane == 1 ? 0.0 : 1.0;
}

/**
 * End cost functions ------------------------------------------------------------------------
 */ 

/**
 * Function which finds the best trajectory for the ego vehicle by generating trajectories and comparing their costs.
 * The lowest cost trajectory is selected.
 * @param vehicle_telemetry - ego vehicle map x, map y, heading, Frenet s, Frenet d
 * @param sensor_fusion - surrounding vehicles map x, map y, vel x, vel y, Frenet s, Frenet d
 */
vector<vector<double>> bestTrajectory(double &vel, int &lane, bool &collision, bool &braking_only,
                              const vector<double> &previous_path_x,
                              const vector<double> &previous_path_y,
                              const vector<vector<double>> &sensor_fusion, 
                              const vector<double> &end_path,
                              const vector<double> &vehicle_telemetry, 
                              const vector<double> &map_waypoints_s,
                              const vector<double> &map_waypoints_x,
                              const vector<double> &map_waypoints_y){


  // Get the current ego vehicle lane (which may be different than the target internally tracked lane)
  int ego_lane = (int)floor((vehicle_telemetry[4] - 1)/4);
  if(ego_lane < 0){
    ego_lane = 0;
  }
  // Find the next states for the vehicle
  vector<State> valid_states = validStates(lane, ego_lane, collision, braking_only);

  vector<vector<vector<double>>> valid_trajectories;
  vector<double> end_velocities;
  vector<double> lane_velocities;
  vector<int> target_lanes;
  vector<double> costs;

  for(const auto &s: valid_states){
    int lane_change = 0;
    switch(s){
      case(State::KL):
        lane_change = 0;
        break;
      case(State::LCL):
        lane_change = -1;
        break;
      case(State::LCR):
        lane_change = 1;
        break;
      default:
        lane_change = 0;
        break;
    }

    double target_d = (double) lane;
    if(target_d < 0){
      target_d = 0;
    }
    // Preserve the lane number only for storing
    double target_d_ = target_d + lane_change; 
    // Convert the target lane to Frenet D coordinate
    target_d = (target_d + lane_change)*4 + 2;
    // Additional logic to ensure valid lane states
    if(target_d >= 12){
      target_d = 10;
    }
    else if(target_d <= 0){
      target_d = 2;
    }

    vector<double> start = {30, target_d};
    vector<double> end = {90, target_d};

    // Temporarily storage for the expected velocity of the new trajectory
    double vel_ = vel;
    double lane_vel = MAX_VEL;
    vector<vector<double>> trajectory_ = generateTrajectory(start, end, vel_, lane_vel, previous_path_x, previous_path_y, 
                                                    sensor_fusion, end_path, vehicle_telemetry, 
                                                    map_waypoints_s, map_waypoints_x, map_waypoints_y);

    valid_trajectories.push_back(trajectory_);
    end_velocities.push_back(vel_);
    lane_velocities.push_back(lane_vel);
    target_lanes.push_back(target_d_);
     
  }

  // Calculate the costs associated with executing each prototype trajectory
  int collision_counter = 0;
  for(int i = 0; i < valid_trajectories.size(); ++i){
    double collision_cost = collisionCost(valid_trajectories[i], sensor_fusion, vehicle_telemetry, target_lanes[i])*pow(10,3);
    double efficieny_cost = efficiencyCost(lane_velocities[i])*40.0;
    double lane_change_cost = laneChangeCost(lane, target_lanes[i])*3.0;
    double lane_occupancy_cost = laneOccupancyCost(target_lanes[i], sensor_fusion, vehicle_telemetry[3])*0.25;
    double center_deviation_cost = centerDeviationCost(target_lanes[i])*0.5;
    double total_cost = collision_cost + efficieny_cost + lane_change_cost + lane_occupancy_cost + center_deviation_cost;
    costs.push_back(total_cost);
    if(collision_cost > 0){
      collision_counter++;
      // if changing lanes set collisions to true so that the validStaets function can allow lane reversion
      if(transitioning){
        collision = true;
      }
    }
    string print_statement;
    switch(valid_states[i]){
      case(State::KL):
        print_statement = "KL ";
        break;
      case(State::LCL):
        print_statement = "LCL";
        break;
      case(State::LCR):
        print_statement = "LCR";
        break;
      default:
        print_statement = "";
        break;
    }
    std::cout << "Total Cost [" << print_statement << "]" << "(" << target_lanes[i] << "): " << total_cost << std::endl;
  }

  // If all trajectories result in a collision, try braking only
  collision_counter == valid_trajectories.size() ? braking_only = true : braking_only = false;

  // Select the lowest cost for execution
  int minElementIndex = std::min_element(costs.begin(),costs.end()) - costs.begin();
  vector<vector<double>> trajectory = valid_trajectories[minElementIndex];
  vel = end_velocities[minElementIndex]; // updating internally tracked velocity corresponding to selected trajectory
  lane = target_lanes[minElementIndex]; // update the internally tracked lane to the target lane
  return trajectory;
}

#endif  // HELPERS_H