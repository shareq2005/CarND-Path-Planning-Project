# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Project Overview
This is a path-planning project for Udacity's self-driving nanodegree program. The goal of the path planner is to safely navigate a car around a virtual highway with other traffic. The car has access to the localization and sensor fusion data. While navigating, the car should closely match a speed of 50 MPH. It should also slow down or change lanes, where needed. In addition, the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Implementation
The implementation is mostly in main.cpp, and closely matches the one provided in Udacity's walkthrough of this project. For smooth trajectory generation, my implementation makes use of cubic splines. 

I have implemented a simple behavior planner, which is implemented as part of two functions GetOtherCarsState() and UpdateCarLaneAndVelocity(). The GetOtherCarState basically iterates through the sensor fusion data of all the cars, and determines if any car is either:
1. Close ahead in the lane
2. In the lane to the left, i.e. left blind spot.
3. In the lane to the right, i.e. right blind spot.

This is done by calculating the projected position of the cars from the sensor fusion data using their speed and distance. The projected position is then compared to the position of our car to determine if it falls within a limit. The function returns parameter of boolean flags to indicate which of the 3 cases above statisfy.

The next step is to update the car's velocity, and change lanes if needed. This is imple The behavior which I implemented in UpdateCarLaneAndVelocity(). Basically, the car will stay in it's lane. If it encounters a slow moving car ahead, then it checks if there are cars in it's left or right lanes. It one of these lanes are free, the car then moves to that lane giving left lane a preference. However, if a lane change is not possible due to a car in it's adjacent lane(s), then the car simply stays in the lane and slows down.

### Testing

I was able to drive the car successfully around the track. Here is an image below which shows the car driven over 6 miles.


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
## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


