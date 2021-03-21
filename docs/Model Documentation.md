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
[image2]: ./examples/grayscale.jpg "Grayscaling"
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

You're reading it! and here is a link to my [project code](https://github.com/rezarajan/sdc-path-planning.git)

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
* (`validStates`) Understanding the next valid lane transitions a vehcile may perform:
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


