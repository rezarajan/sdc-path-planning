# **Path Planning - Highway Driving** 

## Writeup

### The following document outlines the process of creating an autonomous system which navigates a car around a cyclic highway track in a simulator.

---

**Highway Driving Project**

The goals / steps of this project are the following:
* Safely navigate a vehicle (ego vehicle) around a virtual highway, with moving traffic, for at least 1 loop (6846m)
* Avoid collisions with merging traffic
* Optimize lane transitions to overtake slower traffic
* Ensure that the vehicle's speed does not exceed the 50 MPH speed limit
* Ensure that the vehicle's acceleration does not exceed 10m/s^2
* Ensure that the vehicle's jerk does not exceed 10m/s^3
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./images/path_planning_flowchart.png "Path Planning Flowchart"
[image2]: ./images/collision.gif "Collision Example"
[image3]: ./examples/random_noise.jpg "Random Noise"
[image4]: ./examples/placeholder.png "Traffic Sign 1"
[image5]: ./examples/placeholder.png "Traffic Sign 2"
[image6]: ./examples/placeholder.png "Traffic Sign 3"
[image7]: ./examples/placeholder.png "Traffic Sign 4"
[image8]: ./examples/placeholder.png "Traffic Sign 5"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1971/view) and describe how I addressed each point in my implementation.  

---
## Reflection

#### 1. The code model for generating paths is described in detail. This can be part of the README or a separate doc labeled "Model Documentation".

You're reading it! and here is a link to my [project code](https://github.com/rezarajan/sdc-path-planning.git). A video showcasing the car driving for 25 minutes is available [here](https://youtu.be/lzqQZv1BSao).

---

## Code Structure

The implementaion fo the code follows a functional programming style versus an object oriented one. This approach is taken simply because in C++, this is what I am more accustomed to. For the purposes of this project, this method suffices, but an object-oriented approch is perhaps recommended for better flexibility of the code. This is further explained in the recommendations.

Most of the code resides in the `helpers.h` header file, with a single endpoint exposed in the `main.cpp` file. In other words, the logic of path planning is handled by a single function called `bestTrajectory`, which is called in `main.cpp` to return a vector of `[x,y]` path points.

The following are the functions used:

#### Logic Functions
* `validStates` - This function returns the next valid lane change states for the ego vehicle
* `getTargetVelocity` - Find the closest vehicles both ahead and behind the car, in the target lane. Returns the target velocity for path generation.
* `generateTrajectory` - Generates a spline given the start and end positions in Frenet coordinates
* `bestTrajectory` - Function which finds the best trajectory for the ego vehicle by generating trajectories and comparing their costs. The lowest cost trajectory is selected.

#### Cost Functions
* `collisionCost` - Binary cost function which penalizes collisions
* `efficiencyCost` - Cost function which penalizes speeds slower than the maximum velocity
* `laneChangeCost` - Cost function which penalizes changing lanes
* `laneOccupancyCost` - Cost function which penalizes lanes which are occupied by more cars (ahead of ego vehicle)
* `centerDeviationCost` - Cost function which penalizes deviations from the center lane

---

## Path Planning Structure

To understand why the code is separated as outlined, the process of path planning will be described. Path planning in the context of highway driving involves the following:

![image1]

## Behavior

At the highest level, behavior involves determining the next series of valid actions the ego vehicle may perform, given all data. In the code, this will include:
* (`validStates`) Finding the next valid lane transitions a vehcile may perform:
  * Keep Lane
  * Lane Change Left
  * Lane Change Right

## Prediction
* (`getTargetVelocity`) Given a valid lane, determine the maximum and/or minimum speed the ego vehicle should drive at, based on:
  * The speed limit
  * Any vehicles ahead in the target lane driving slower than the ego
  * Any vehicles behind in the target lane driving faster than the ego

## Trajectory
* (`generateTrajectory`) Given the vehicles's current position, the target lane, and the target velocity from the above functions, generate a trajectory for the vehicle to follow, and which obeys the constrains of speed, acceleration and jerk.
* (`bestTrajectory`) For all `validStates`, `getTargetVelocity` and `generateTrajectory`. Then, compute the cost associated with each trajectory using weighted cost functions. Select the trajectory with the lowest cost and return that to the motion control system. In other words, generate prototype trajectories, and select the best one.

---

## Explaning Cost Functions

#TODO:

---
## Addressing Rubric Points

The code which addresses each rubric point will be described below:

#### 1. The car is able to drive at least 4.32 miles without incident.
Valid trajectories are generated such that the ego vehicle is able to complete 4.32 miles around the track, without indcident in most cases. [Here](https://youtu.be/lzqQZv1BSao) is a link to a video showcasing the vehicle driving for 25 minutes, without collision.

However, there is still an issue with the current implementation (or perhaps this may be considered an edge case), in which the simulator merges a vehicle within a very close distance to the car. A more sophisticated prediction system (perhaps using machine learning techniques) may be required to handle such situations.

The following clip shows an instance of a sudden merge, which the current prediction system fails to recognize in time, resulting in a collision:

![image2]

#### 2. The car drives according to the speed limit.
The car drives as close as possible to the speed limit, when conditions permit it. This is managed by the `getTargetVelocity` function.

#### 3. Max Acceleration and Jerk are not Exceeded.
The vehicle speed is managed by the `getTargetVelocity` function, such that the generated path (spline) meets the acceleration and jerk requirements.

#### 4. Car does not have collisions.
Barring the edge case described [above](#1-the-car-is-able-to-drive-at-least-432-miles-without-incident), the car avoids collisions by managing speed and switching to lanes which avoid collisions. This is done by using the `collisionCost` cost function, which assigns very high costs for trajectories which result in collisions.

#### 5. The car stays in its lane, except for the time between changing lanes.
The car does not spend more than 3 seconds during a lane transition, and stays within the lane when keeping lanes (right hand side of the road only). Lane changes are based on the `laneChangeCost`, `laneOccupancyCost` and `centerDeviationCost` cost functions, which state that staying in the center lane is preferred, unless either of the other lanes are clear and have higher speed limits.

#### 6. The car is able to change lanes.
The car changes lanes based on a series of cost functions.
