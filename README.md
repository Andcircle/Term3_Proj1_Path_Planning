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
if there is no car in certain range ahead of current lane, then front clear
if in left lane, there is no car within certain range both before and after current car's longitudinal postion, then left clear.
if in right lane, there is no car within certain range both before and after current car's longitudinal postion, then left clear.

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

If 


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





