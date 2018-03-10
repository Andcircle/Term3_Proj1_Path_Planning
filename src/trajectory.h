#include <iostream>
#include <algorithm>
#include "spline.h"
#include "help.h"

class Trajectory
{
public:

	vector<double> map_xs;
	vector<double> map_ys;
	vector<double> map_ss;
	vector<double> map_dxs;
	vector<double> map_dys;

	const double spd_limit = 50 * 0.44; //m/s
	const double acc_limit = 9; // m/s^2
	const double step_t = 0.02;
	const double safe_dis = 30;
	const int horizon = 50;

	double cur_spd;

	Trajectory();
	~Trajectory();

	void init_map(const vector<double> & map_waypoints_x, const vector<double> & map_waypoints_y, const vector<double> & map_waypoints_s,
			const vector<double> & map_waypoints_dx,const vector<double> & map_waypoints_dy, double max_s);

	void calc_trajectory(const double car_x,const double car_y,const double car_s,
		const double car_d,const double car_yaw,const double car_spd, const double end_s,const double end_d,
		const vector<double> & pre_xs, const vector<double> & pre_ys, const vector<vector<double>> & sensor_fusion,
		vector<double> & next_x_vals,vector<double> & next_y_vals);

	void behavior_plan(const double car_s, const double car_d,const double car_spd,
			const vector<vector<double>> & sensor_fusion, int & tgt_lane,double & tgt_spd);
};
