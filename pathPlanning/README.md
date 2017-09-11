# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Solution

The solution to this project consists of the following components:
* Path Planning - receives as input previous path, localization data and position 
of all other vehicles on the same side of the road and produces a list of next 50 points that
is fed back to the simulator
* Waypoints - loads the list of points that describes the highway/track and provides methods to 
convert between Cartesian and Frenet coordinates. Most of the code in this class was already 
provided with the started code. My only contribution was the method that converts from Frenet 
to Cartesian coordinates
* Vehicle - a simple data holder for vehicle data provided by sensor fusion
* Traffic - uses the list of vehicles as an input and calculates distance to nearest 
vehicles and also the speed for each lane. This class is used by the Path Planner to decide
what lane to use.
* Main  - acts as a bridge between Simulator and Path Planning

## Implementation Details

### Analyzing Traffic data
The first step is to analyze the traffic data provided by the simulator. In this analysis we 
only include vehicles within a search radius (currently set at 50 meters). As part of this analysis
we calculate the following:
* distance to nearest vehicle in front of out car for each lane
* distance to the nearest vehicle behind our car for each lane
* speed for each lane. Lane speed is given by the slowest vehicle on that lane that is in front of our car.

Also note that since the simulator provides us with previous points that were not already processed
we need to adjust vehicles s values to account for this time offset. For example if we receive 10 previous
points we predict each vehicle's position after 0.02 * 10 = 200 ms. This is pretty simple actually since 
we can assume the velocity to be constant within that time interval.

```json
s_final = s_initial + speed * number_of_previous_points * 0.02
```

### Finding the best lane
The next step is to find the best lane. In order to do that we calculate a cost for each of the lanes 
and we pick the one with the lowest cost. To calculate the cost we include the following factors:
* we add a penalty for changing the lane
* we add a very big penalty for leaving the road
* we add a very big penalty when trying to change the lane when there are vehicles that are too close on the destination lane. We do this in order to prevent collisions.
* we also allow changing two lanes but first we check that the lane in the middle does not have vehicles close to us
* we add a penalty for slow lanes
* we add a penalty for vehicles in front of us

The code for the cost calculator is in PathPlanner::calculateCost()

### Calculate Path
The last step is to actually calculate the path to transition to this lane. The goal is to generate a list of points that we feed back into the simulator.

The Path Planner keeps track of the following path data (for all state maintained by Path Planner check PathPlanner.h):
 * vectors of next 50 points for x,y,s and d
 * list of previous x and y points received from the simulator
 
We first clean from the list of next points all the points that were already processed by the simulator (these are the points NOT in the list of previous points).
 
We then calculate a d spline in order to generate a smooth transition to the target lane. 
 
Next we add new points to the list of existing points until we end up with the desired number of points.
 
The code that performs these calculations is in PathPlanner::recalculate_path()

Below is a video recording of the car driving 5 miles without incident.

[![Path Planning](https://img.youtube.com/vi/t7P4au6bT1Q/0.jpg)](https://youtu.be/t7P4au6bT1Q)
    
 
   
 

