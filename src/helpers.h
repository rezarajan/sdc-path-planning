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

vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;
    
  MatrixXd B = MatrixXd(3,1);     
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];
          
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;
  
  vector <double> result = {start[0], start[1], .5*start[2]};

  for(int i = 0; i < C.size(); ++i) {
    result.push_back(C.data()[i]);
  }

  return result;
}

double toEquation(vector<double> &jmt, double time){
  double total = 0.0;
  for(int i = 0; i < jmt.size(); ++i){
    total += jmt.data()[i] * pow(time,i);
  }
  return total;
}

/**
 * @param car_d - ego vehicle Frenet d value
 * 
 * This function returns the next valid lane change states for the ego vehicle
 */ 
vector<State> validStates(const double &car_d){
  // Find the next states for the vehicle
  vector<State> valid_states;
  switch(int(car_d/4)) {
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

  return valid_states;
}


 /**
   * Find the closest vehicles both ahead and behind the car, in the target lane. Returns the target velocity
   * for path generation.
   * @param sensor_fusion sensor fusion data {car id, map x, map y, velocity x, velocity y, Frenet s, Frenet d}
   * @param ref_x reference point x on path (usually at the end point of the previous path if there is one)
   * @param ref_y reference point y on path (usually at the end point of the previous path if there is one)
   * @param car_s Frenet s value for corresponding reference point on path (ref_x, ref_y -> s)
   * @param target vector of Frenet {s, d} coordinates, where s is relative to the reference point
  */ 
double getTargetVelocity(const vector<vector<double>> &sensor_fusion, const double &ref_x, const double &ref_y,
                                            const double &car_s, const vector<double> &target){

  // Velocity Control Logic
    const double MAX_ACCEL = 10;
    const double MAX_VEL = 49.5 * 0.44704;
    const double ACCEL_TIME = 0.01; // Acceleration is measured in 0.2s invervals by the simulator, but the messages update every 0.02s (0.02/0.2 = 0.1 for proper scaling)
    const double VEL_BUFFER = MAX_ACCEL*ACCEL_TIME; // Velocity buffer to ensure car stays within limits with controller error


    // Find all vehicles in the target lane
    vector<vector<double>> target_vehicles;
    int target_d = floor(target[1]/4);
    for(const auto &s: sensor_fusion){
      int s_d = floor(s[6]/4);
      if(s_d == target_d){
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
    /**
     *  TODO: move target id caching to another function for collision checking.
     *        This is not required here.
     */ 
    bool car_ahead = false;
    bool car_behind = false;
    vector<int> target_ids;
    double target_velocity = MAX_VEL;
    for(int i = 0; i < target_vehicles.size(); ++i){
      if(car_ahead && car_behind){
        break;
      }
      if(!car_ahead && (target_vehicles[i][1] > car_s)){
        target_ids.push_back(i);
        // Set a minimum target velocity if there is a car ahead, and in range of target distance
        if((target_vehicles[i][2] <= MAX_VEL) && (target_vehicles[i][0] < target[0])){
          target_velocity = target_vehicles[i][2];
        }
      }
      if(!car_behind && (target_vehicles[i][1] <= car_s)){
        target_ids.push_back(i);
      }
    }

      return target_velocity;
}

  /**
   * Generates a spline given the start and end positions in Frenet coordinates
   * @param start starting Frenet coordinates as a vector of {s, d} relative to the ego vehicle
   * @param end end Frenet coordinates as a vector of {s, d} relative to the ego vehicle
   * @param previous_path previous path points to include in trajectory generation in map {x,y} coordinates
   * @param end_path the Frenet coordinates of the end of the previous path {s,d}
   * @param vehicle_telemetry ego vehicle map x, map y, heading, Frenet s, Frenet d
   * @param sensor_fusion sensor fusion data {car id, map x, map y, velocity x, velocity y, Frenet s, Frenet d}
   * @param map_waypoints vector of map waypoint {Frenet s, x, y} coordinates
  */ 
vector<vector<double>> generateTrajectory(const vector<double> &start, const vector<double> &end, double &vel,
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

    // Preventing collisions.
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

    // Points of reference from car coordinates to map coordinates
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


    tk::spline s;
    s.set_points(spline_points_x, spline_points_y);


    for (int i = 0; i < path_size; ++i) {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }


    double target_x = 30.0; // some target in the future
    double target_y = s(target_x); // some target in the future
    double target_dist = distance(target_x, target_y, 0.0, 0.0);


    // Velocity Control Logic
    const double MAX_ACCEL = 10;
    const double MAX_VEL = 49.5 * 0.44704;
    const double ACCEL_TIME = 0.01; // Acceleration is measured in 0.2s invervals by the simulator, but the messages update every 0.02s (0.02/0.2 = 0.1 for proper scaling)
    const double VEL_BUFFER = MAX_ACCEL*ACCEL_TIME; // Velocity buffer to ensure car stays within limits with controller error


    // Velocity Limiter for Lane Keeping
    vector<double> target = {target_dist, 6};
    double target_vel = getTargetVelocity(sensor_fusion, ref_x, ref_y, car_s, target);


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
    vector<vector<double>> trajectory = {next_x_vals, next_y_vals};
    return trajectory;
}

/**
 * @param vehicle_telemetry - ego vehicle map x, map y, velocity, heading, Frenet s, Frenet d
 * @param sensor_fusion - surrounding vehicles map x, map y, vel x, vel y, Frenet s, Frenet d
 */
vector<State> bestTrajectory(const vector<double> &vehicle_telemetry, const vector<vector<double>> &sensor_fusion){


  // Find the next states for the vehicle
  vector<State> valid_states = validStates(vehicle_telemetry[6]);

  /**
   * TODO: Generate trajectories for each valid state and find the cheapest one (based on cost functions)
   * TODO: Create cost functions for each trajectory
   */
  State best_state = State::KL;
  vector<State> best_states;
  return best_states;
}

#endif  // HELPERS_H