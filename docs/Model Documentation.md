# **Path Planning - Highway Driving** 

## Writeup

### The following document outlines the process of creating path planner which navigates a car around a cyclic highway track in a simulator.

---

## Highway Driving Project

The goals / steps of this project are the following:
* Safely navigate a vehicle (ego vehicle) around a virtual highway, with moving traffic, for at least 1 loop (6846m)
* Avoid collisions with merging traffic
* Optimize lane transitions to overtake slower traffic
* Ensure that the vehicle's speed does not exceed the 50 MPH speed limit
* Ensure that the vehicle's acceleration does not exceed 10ms<sup>-2</sup>
* Ensure that the vehicle's jerk does not exceed 10ms<sup>-3</sup>
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./images/path_planning_flowchart.png "Path Planning Flowchart"
[image2]: ./images/25_min.gif "25 Minutes of Collision-Free Driving"
[image3]: ./images/collision.gif "Collision Example"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1971/view) and describe how I addressed each point in my implementation.  

---
## Reflection

#### 1. The code model for generating paths is described in detail. This can be part of the README or a separate doc labeled "Model Documentation".

You're reading it! and here is a link to my [project code](https://github.com/rezarajan/sdc-path-planning.git). A video showcasing the car driving for 4.32 miles, in five trials, is available [here](https://youtu.be/VFb8rHQNdyM).

---

## Code Structure

The code implementaion follows a functional programming style versus an object-oriented one. This approach is taken simply because in C++ this is what I am more accustomed to. For the purposes of this project this method suffices, but an object-oriented approch is perhaps recommended for better code flexibility, for e.g. if implementing the [recommendations](#recommendations).

Most of the code resides in the `helpers.h` header file, with a single endpoint exposed in the `main.cpp` file. In other words, the logic of path planning is handled by a single function called `bestTrajectory`, which is called in `main.cpp` to return a vector of `[x,y]` path points.

The following are the functions used:

#### *Logic Functions*
* `validStates` - This function returns the next valid lane change states for the ego vehicle
* `getTargetVelocity` - Finds the closest vehicles both ahead and behind the car, in the target lane. Returns the target velocity for path generation.
* `generateTrajectory` - Generates a spline given the start and end positions in Frenet coordinates
* `bestTrajectory` - Function which finds the best trajectory for the ego vehicle by generating prototype trajectories and comparing their costs. The lowest cost trajectory is selected.

#### *Cost Functions*
* `collisionCost` - Binary cost function which penalizes collisions
* `efficiencyCost` - Cost function which penalizes speeds slower than the maximum speed
* `laneChangeCost` - Cost function which penalizes changing lanes
* `laneOccupancyCost` - Cost function which penalizes lanes which are occupied by more cars (ahead of ego vehicle)
* `centerDeviationCost` - Cost function which penalizes deviations from the center lane

*Cost functions are explained in more detail [below](#explaning-cost-functions).*

---

## Path Planning Structure

To understand why the code is separated into the above defined functions, the process of path planning will be described. Path planning in the context of highway driving involves the following:

![image1]

## Behavior

At the highest level, behavior involves determining the next series of valid actions the ego vehicle may perform, given localization and prediction data. In the code, this will include:
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
* (`generateTrajectory`) Given the vehicles's current position, the target lane, and the target velocity from the above functions, generate a trajectory for the vehicle to follow, which obeys the constrains of speed, acceleration and jerk.
* (`bestTrajectory`) For all `validStates`, `getTargetVelocity` and `generateTrajectory`. Then, compute the cost associated with each trajectory using weighted cost functions. Select the trajectory with the lowest cost and return that to the motion control system. In other words, generate prototype trajectories, and select the best one.

---

## Explaning Cost Functions

Cost functions serve as a way for the path planner to determine the most desired behavior for the ego vehicle and as such, these functions generally come under the [behavior module](#path-planning-structure). However, cost functions are not strictly defined, i.e. it is not trivial to determine a specific combination of cost functions which will lead to perfect behavior in all situations. However, a bit of logic is used in determining a general set of costs which a human may typically consider when driving. These cost functions ensure the behavior of the ego vehicle is intentional in as many situations as possible. In my implementation these include:

* ***Collision Cost:*** trajectories which collide with other vehicles will incur a significant cost. This is important, as collisions are the most undesirable result when driving. A weight of 1000 is set for the `collisionCost` function, to ensure any path which results in collison is rejected.
* ***Efficiency Cost:*** the most desirable behavior is for the car to travel as close to the speed limit as possible, overtaking slower cars. Therefore, the `efficiencyCost` function rewards trajectories with higher speeds.
* ***Lane Change Cost:*** when a human drives they don't haphazardly change lanes. A lane change must be intentional, and occurs usually when overtaking or approaching a waypoint. Therefore, to prevent the planner from constantly changing lanes, the `laneChangeCost` function checks for trajectories which require a lane change, and penalize them with a weighted cost of 3.0. Therefore, to change lanes in this implementation, the target lane must result in a meaningfully higher speed.
* ***Lane Occupancy Cost:*** typically, it is easier to drive in lanes which contain less traffic. Furthermore, generally the chances of moving at higher speeds increase when lanes have less obstacles in front of the car. Therefore, the `laneOccupancyCost` function adds a small cost to each trajectory for each vehicle in the target lane.
* ***Center Deviation Cost:*** this function is a bit less intuitive, but is still useful in the simulator environment. Essentially, the `centerDeviationCost` function rewards the vehicle for staying in the center lane. This behavior is desired in the three-lane highway scenario, since there are more options for the vehicle to overtake other traffic using either the left or right lanes, and get ahead of traffic. This cost opposes the lane change cost for the center lane, but adds to it for other lanes. To ensure that the center lane is not too biased, however, this cost is relatively low compared to the lane change cost, and therefore the center lane must still result in higher speeds and less traffic for the car to return to/stay at center.

---
## Addressing Rubric Points

The code which addresses each rubric point will be described below:

#### *1. The car is able to drive at least 4.32 miles without incident.*
Valid trajectories are generated such that the ego vehicle is able to complete 4.32 miles around the track, without indcident in most cases. [Here](https://youtu.be/VFb8rHQNdyM) is a link to a video showcasing the vehicle driving for 4.32 miles in five trials, without collision. Furthermore, a snippet of the vehicle driving for 25 minutes without collision is shown below:

![image2]

However, there is still an issue with the current implementation (or perhaps this may be considered an edge case), in which the simulator merges a vehicle within a very close distance to the car. A more sophisticated prediction system (perhaps using machine learning techniques) may be required to handle such situations.

The following clip shows an instance of a sudden merge, which the current prediction system fails to recognize in time, resulting in a collision:

![image3]

#### *2. The car drives according to the speed limit.*
The car drives as close as possible to the speed limit, when conditions permit it. This is managed by the `getTargetVelocity` function.

#### *3. Max Acceleration and Jerk are not Exceeded.*
The vehicle speed is managed by the `getTargetVelocity` function, such that the generated path (spline) meets the acceleration and jerk requirements.

#### *4. Car does not have collisions.*
Barring the edge case described [above](#1-the-car-is-able-to-drive-at-least-432-miles-without-incident), the car avoids collisions by managing speed and switching lanes only when conditions permit. This is done by using the `collisionCost` cost function, which assigns very high costs for trajectories which result in collisions.

#### *5. The car stays in its lane, except for the time between changing lanes.*
The car does not spend more than 3 seconds during a lane transition, and stays within the lane when keeping lanes (right hand side of the road only). Lane changes are based on the `laneChangeCost`, `laneOccupancyCost` and `centerDeviationCost` cost functions, which state that staying in the center lane is preferred, unless either of the other lanes are clear and have higher speed limits.

#### *6. The car is able to change lanes.*
The car changes lanes based on a series of cost functions.

---

## Recommendations

* The current implementation still results in collisions in some situations. To address such issues it is recommended that a more sophisticated prediction system be used, which consideres nuanced movements of surrounding vehicles, and generates more accurate prototype trajectories to check collisions against.
* The current implementation still violates acceleration constraints when changing lanes at full speed around a turn. It is recommended that the `getTargetVelocity` function be updated to check for centripetal acceleration by finding the radius of curvature of the road. Note that this will come with more computational expense.
* It may be benificial for the planner to track gaps between vehicles, to better match speed and position for lane merges. Tracking is currently done based on individual vehicles, which is not always representative of the vehicle's ability to merge into another lane.
* In the left or rightmost lanes, the planner only looks for a single lane change either right or left, respectively. If, for instance, the ego vehicle is in the leftmost lane moving slower than the speed limit, but the rightmost lane is free, it does not look for the opportunity to get into the rightmost lane. This will only occur if the center lane is also moving faster than the speed of traffic. It is recommended that all lanes be checked for potential to overtake traffic.

## Conclusions

A path planner has been created, which navigates an ego vehicle around a simulator highway for at least 4.32 miles in accordance with the [project specifications](#highway-driving-project), except in some edge case situations. The planner is for the most part, robust, and optimizes lane changes through the use of cost functions. Recommendations are also provided for further improvement of the path planner.