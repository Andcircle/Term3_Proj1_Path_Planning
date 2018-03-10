#include "trajectory.h"

Trajectory::Trajectory(){}

Trajectory::~Trajectory(){}

void Trajectory::init_map(const vector<double> & map_waypoints_x, const vector<double> & map_waypoints_y, const vector<double> & map_waypoints_s,
		const vector<double> & map_waypoints_dx,const vector<double> & map_waypoints_dy, double max_s)
{
	  map_xs = map_waypoints_x;
	  map_ys = map_waypoints_y;
	  map_ss = map_waypoints_s;
	  map_dxs = map_waypoints_dx;
	  map_dys = map_waypoints_dy;

	  cur_spd = 0;
}

void Trajectory::calc_trajectory(const double car_x,const double car_y,const double car_s,
		const double car_d,const double car_yaw,const double car_spd, const double end_s,const double end_d,
		const vector<double> & pre_xs, const vector<double> & pre_ys, const vector<vector<double>> & sensor_fusion,
		vector<double> & next_x_vals,vector<double> & next_y_vals)
{

    double pos_x;
    double pos_y;
    double angle;
    int path_size = pre_xs.size();
    //cout<<"pre_path "<<pre_xs.size()<<"\n";

    vector<double> temp_x;
    vector<double> temp_y;

    vector<double> sxs;
    vector<double> sys;



    for(int i = 0; i < path_size; i++)
    {
    	next_x_vals.push_back(pre_xs[i]);
    	next_y_vals.push_back(pre_ys[i]);
    }

    if(path_size == 0)
    {
        pos_x = car_x;
        pos_y = car_y;
        angle = deg2rad(car_yaw);
    }
    else
    {
        pos_x = pre_xs[path_size-1];
        pos_y = pre_ys[path_size-1];

        double pos_x2 = pre_xs[path_size-2];
        double pos_y2 = pre_ys[path_size-2];
        angle = atan2(pos_y-pos_y2,pos_x-pos_x2);

        sxs.push_back(pos_x2);
        sys.push_back(pos_y2);

//        cout<<"new cycle:"<<"\n";
//        cout<<"pre_start: "<<pre_xs[0];
//        cout<<"pre_end: "<<pre_xs[path_size-1];

    }

    sxs.push_back(pos_x);
    sys.push_back(pos_y);

//	cout<<"|car_s: "<<car_s<<"\n";
//	cout<<"|px: "<<pos_x<<"\n";
//	cout<<"|py: "<<pos_y<<"\n";

	int tgt_lane = 1;
	double tgt_spd = 0;

	behavior_plan(car_s, car_d,car_spd,sensor_fusion, tgt_lane, tgt_spd);

//	tgt_lane = 1;

	double s = 0;
	double step = cur_spd*step_t;

	vector<double> xs;

	for (int i=0; i<horizon-path_size; i++)
	{
		if (cur_spd<tgt_spd-1)
		{
			step = cur_spd*step_t+0.5*step_t*step_t*acc_limit;
			cur_spd += step_t*acc_limit;
		}
		else if (cur_spd>tgt_spd)
		{
			step = cur_spd*step_t-0.5*step_t*step_t*acc_limit;
			cur_spd -= step_t*acc_limit;
		}
		//cout<<"step"<<step<<"\n";
		s+=step;

		xs.push_back(s);
	}

	double tgt_d = tgt_lane*4+2;
    vector<double> cur_sd = getFrenet(pos_x, pos_y, angle, map_xs, map_ys);

	vector<double> spl0 = getXY(cur_sd[0] + safe_dis*1, tgt_d, map_ss, map_xs, map_ys);
	vector<double> spl1 = getXY(cur_sd[0] + safe_dis*2, tgt_d, map_ss, map_xs, map_ys);
	vector<double> spl2 = getXY(cur_sd[0] + safe_dis*3, tgt_d, map_ss, map_xs, map_ys);

	sxs.push_back(spl0[0]);
	sxs.push_back(spl1[0]);
	sxs.push_back(spl2[0]);

	sys.push_back(spl0[1]);
	sys.push_back(spl1[1]);
	sys.push_back(spl2[1]);

	double cospsi = cos(-angle);
	double sinpsi = sin(-angle);
	double dx = 0;
	double dy = 0;

	for (int i=0; i<sxs.size(); i++)
	{
//		cout<<"|x: "<<sxs[i];
//		cout<<"|y: "<<sys[i];

		dx = sxs[i] - pos_x;
		dy = sys[i] - pos_y;

		sxs[i] = dx * cospsi - dy * sinpsi;
		sys[i] = dy * cospsi + dx * sinpsi;

//		cout<<"|sx: "<<sxs[i];
//	    cout<<"|sy: "<<sys[i];
//	    cout<<"/////";
	}

//	cout<<"\n";

	tk::spline spl;
	spl.set_points(sxs,sys);

	cospsi = cos(angle);
	sinpsi = sin(angle);

//	cout<<"|angle: "<<angle;
//	cout<<"|px: "<<pos_x;
//	cout<<"|py: "<<pos_y;
//	cout<<"\n";

    double x;
    double y;

	for (int i=0; i<xs.size(); i++)
	{
		x = xs[i];
		y = spl(x);

//		cout<<"|sx: "<<x;
//		cout<<"|sy: "<<y;

		dx = x * cospsi - y * sinpsi;
		dy = y * cospsi + x * sinpsi;

		x = dx + pos_x;
		y = dy + pos_y;

		next_x_vals.push_back(x);
		next_y_vals.push_back(y);

//		cout<<"|x: "<<x;
//		cout<<"|y: "<<y;
//		cout<<"/////";
	}
//	cout<<"new_start: "<<next_x_vals[path_size];
//	cout<<"new_end: "<<next_x_vals[next_x_vals.size()-1];
//	cout<<"\n";

//	cout<<"new_path "<<next_x_vals.size()<<"\n";

}

void Trajectory::behavior_plan(const double car_s, const double car_d,const double car_spd,
		const vector<vector<double>> & sensor_fusion, int & tgt_lane,double & tgt_spd)
{
	int cur_lane = ceil(car_d/4)-1;
	int lane = 0;
	double ahead_spd = -1;
	int s = 0;

	// [ id, x, y, vx, vy, s, d]
	bool front_clear = true;
	bool right_clear = true;
	bool left_clear = true;

	for (int i = 0; i<sensor_fusion.size(); i++)
	{
		lane = ceil(sensor_fusion[i][6]/4)-1;
		s = sensor_fusion[i][5];

		if (cur_lane==0)
			left_clear = false;
		if (cur_lane==2)
			right_clear = false;

		if (lane == cur_lane && s-car_s>0 && s-car_s<safe_dis)
		{
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];

			ahead_spd = sqrt(vx*vx+vy*vy);
			front_clear = false;
		}
		else if (lane == cur_lane-1 && abs(s-car_s)<safe_dis)
			left_clear = false;
		else if (lane == cur_lane+1 && abs(s-car_s)<safe_dis)
			right_clear = false;
	}

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

}















