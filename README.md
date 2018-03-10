# Term3_Proj1_Path_Planning
Design and implement a highway path planner


The target of this project is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic.

## Content of source code
- `src` source code directory:
  - `main.cpp` - communicate with simulation tool, call functions to load map file, and call path planner to calculate trajectory.
  - `trajectory.cpp` - the implemetation of path planner, decide next behavior according to environment, and then calulate trajectory accordingly.
  - `help.cpp` - include all fundamental math function like getXY, getFrenet, etc.
  - `spline.h` - a useful tool for spline interpolation.

## Behavior Plan

The behavior planner is impemented in Trajectory::behavior_plan function.

First, it will analysis the environment:

If there is no car in certain range ahead of current lane, then front clear

If in left lane, there is no car within certain range both before and after current car's longitudinal postion, then left clear.

If in right lane, there is no car within certain range both before and after current car's longitudinal postion, then right clear.


Then the simple behavior planning works as following:

	if (front_clear)
	{
		tgt_spd = spd_limit;
		tgt_lane = cur_lane;
	}
	else if (!left_clear&&!right_clear)
	{
		tgt_spd = ahead_spd;
		tgt_lane = cur_lane;
	}
	else if (left_clear)
	{
		tgt_spd = spd_limit;
		tgt_lane = cur_lane-1;
	}
	else if (right_clear)
	{
		tgt_spd = spd_limit;
		tgt_lane = cur_lane+1;
	}


## Trajectory Calculation

This logic is implemented in function Trajectory::calc_trajectory.

In the map data, there is huge interval between way points, that's why, even for just following the road without changing lanes, we still need a curve fitting or interpolation tool, here we use the spline interpolation tool. 

After behavior plan, we got target speed and target lane, then we use last 2 points of previous planned path, and another 2~3 points in target lane with the same relatively large distance interval, to generate the spline curve. This curve already take changing lane into consideration. 

To be noted: only the last one point of previous path is not enough, because previous path will not get enough weights, when changing lanes, the new spline curve will change direction abruptly.

With target speed and stored current speed, we can calculate all the x value of the path points in car local coordinates, then use existed spline curve for interpolation to get y value. After that, all these path points can be transferred to global coordinates and send to simulator.

To be noted:
1. Acceleration limit should be taken into consideration.
2. According to experiments, the simulator can only take 2~3 path points in every loop, so the car_speed should not be used to update speed in next loop, instead, the speed of last point in previous path has to be stored for this purpose.
3. GetFrenet and GetXY is just a high level approximation, given x, y, after GetFrenet and then GetXY, the value will be completely different, so certain logic can not be used.


## How to run the code
Clone this repo, 
Since Eigen3.3 has too many files, so it hasn't been included in this repo.
Pls add Eigen folder to src/
And then:
```
mkdir build && cd build
cmake ../src/  && make
./mpc
```





