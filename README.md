# CarND-Path-Planning-Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)

[video0]: ./path32.gif "Final video"

<p align="center">
	<img src="/write_up_images/path32.gif" alt="Video Output"
	title="Video Output"  />
</p>


   
### Simulator.
In this project the Term3 Simulator will be used [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Car's localization and sensor fusion data will be provided. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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
### Generate Paths

This project was based on the Project Q&A provided by Aaron Brown and David Silver: https://classroom.udacity.com/nanodegrees/nd013/parts/01a340a5-39b5-4202-9f89-d96de8cf17be/modules/b74b8e43-47d1-47d6-a4cf-4d64ea3e0b80/lessons/407a2efa-3383-480f-9266-5981440b09b3/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d

#### 1) Helper Functions

##### a) double deg2rad(double x)
Conversion from degrees to radians 


##### b) double rad2deg(double x)
Conversion from radians to degrees

##### c) double distance(double x1, double y1, double x2, double y2)
Euclidean distance between two points

##### d) int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
Calculate closest waypoint to current x, y position
	
##### e) int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
Returns next waypoint of the closest waypoint

##### f) vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
Transform from Cartesian x,y coordinates to Frenet s,d coordinates

##### g) vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
Transform from Frenet s,d coordinates to Cartesian x,y

#### 2) Spline Interpolation

<p align="center">
	<img src="/write_up_images/spline_interpolation.png" alt="Spline Interpolation"
	title="Spline Interpolation"  />
</p>
