#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#define MAX_SPEED_LIMIT              49.5
#define DECELERATE                   0.224  //Equivalent to 5 m/s^2
#define ACCELERATE                   0.4032 //Equivalent to 9 m/s^s
#define NEXT_LANE_FRONT_CAR_DIST_MAX 100
#define FRONT_CAR_DIST_MAX           30
#define NEXT_LANE_FRONT_CAR_DIST_MIN 30
#define NEXT_LANE_BACK_CAR_DIST_MIN  50 //30

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  /*Yo keep track of which lane the car is in. lane=0: Left Lane, lane=1: Middle Lane, lane=2: Right Lane*/
  int lane=1; 

  /*ref val is the Less than the maximum velocity the car is allowed to have - in miles/hour */
  double ref_vel = 0.0;
  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            /*The number of unused waypoints from the previous state*/
            int prev_path = previous_path_x.size();
            
            if (prev_path > 0)
            {
              car_s = end_path_s;
            }

            bool too_close = false;
            double front_car_vel = 0.0;

            /*Iterate through all theother  cars from the sensor fusion data*/
            for (int i = 0; i < sensor_fusion.size(); i++)
            {
              /*Check if the car is in my lane*/
              double carn_d = sensor_fusion[i][6];
              if ((carn_d < (2+4*lane+2)) && (carn_d > (2+4*lane-2)))
              {
                double carn_vx = sensor_fusion[i][3];
                double carn_vy = sensor_fusion[i][4];
                double carn_vel = sqrt(carn_vx*carn_vx + carn_vy*carn_vy);
                double carn_s = sensor_fusion[i][5];
                
                /*Predict the other car's future location*/
                carn_s += ((double)prev_path*0.02*carn_vel);
                /*Check if the front car is a safe distance away from self car in the future*/
                if ((carn_s > car_s) && ((carn_s-car_s) < FRONT_CAR_DIST_MAX))
                {
                  front_car_vel = carn_vel;
                  too_close = true;
                }
              
              }
            }

            /*If the front car is too close to self car, then check if lane change is possible*/
            if (too_close)
            {
              bool left_too_close;
              bool right_too_close;
              int left_car_cnt;
              int right_car_cnt;
              double left_close_car_dist;
              double right_close_car_dist;
              double left_close_car_vel;
              double right_close_car_vel;
              double min_left_car_vel;
              double min_right_car_vel;
              int lane_delta = 0;
              int next_lane = lane;
              if ((lane == 1) || (lane == 2))
              {
                left_too_close = false;
                left_close_car_dist = 0.0;
                left_close_car_vel = 0.0;
                left_car_cnt = 0;
                lane_delta = -1;
                /*For left lane change*/
                for (int i = 0; i < sensor_fusion.size(); i++)
                {
                  /*Is the front car in the left lane too close*/
                  double carn_d = sensor_fusion[i][6];
                  if ((carn_d < (2+4*(lane+lane_delta)+2)) && (carn_d > (2+4*(lane+lane_delta)-2)))
                  {
                    double carn_vx = sensor_fusion[i][3];
                    double carn_vy = sensor_fusion[i][4];
                    double carn_vel = sqrt(carn_vx*carn_vx + carn_vy*carn_vy);
                    double carn_s = sensor_fusion[i][5];

                    /*Predict the other car's future location*/
                    carn_s += ((double)prev_path*0.02*carn_vel);
                    if (((carn_s > car_s) && ((carn_s-car_s) > NEXT_LANE_FRONT_CAR_DIST_MIN)))
                    {
                      /*Get the closest front car in the left lane and its velocity*/
                      /*Get the slowest car velocity in the left lane*/
                      if(left_car_cnt == 0)
                      {
                        min_left_car_vel = carn_vel;
                        left_close_car_dist = carn_s-car_s;
                        left_close_car_vel = carn_vel;
                        left_car_cnt++;
                      }
                      else
                      {
                        if(carn_vel < min_left_car_vel)
                          min_left_car_vel = carn_vel;
                        if((carn_s-car_s) < left_close_car_dist)
                        {
                          left_close_car_dist = carn_s-car_s;
                          left_close_car_vel = carn_vel;
                        }
                      }
                    }
             
                     /*Check if the front and back cars in the left lane are at safe distance away from self car in the future*/       
                    if (((carn_s > car_s) && ((carn_s-car_s) < NEXT_LANE_FRONT_CAR_DIST_MIN)) || ((carn_s < car_s) && ((car_s-carn_s) < NEXT_LANE_BACK_CAR_DIST_MIN)))
                    {
                      left_too_close = true;
                    }
                  }
                }
                
                if(left_too_close == false)
                  next_lane = lane+lane_delta;
              }
              
              if ((lane == 0) || (lane == 1))
              {
                right_too_close = false;
                right_close_car_dist = 0.0;
                right_close_car_vel = 0.0;
                right_car_cnt = 0;
                lane_delta = 1;
                /*For right lane change*/
                for (int i = 0; i < sensor_fusion.size(); i++)
                {
                  /*Is the front car in the right lane too close*/
                  double carn_d = sensor_fusion[i][6];
                  if ((carn_d < (2+4*(lane+lane_delta)+2)) && (carn_d > (2+4*(lane+lane_delta)-2)))
                  {
                    double carn_vx = sensor_fusion[i][3];
                    double carn_vy = sensor_fusion[i][4];
                    double carn_vel = sqrt(carn_vx*carn_vx + carn_vy*carn_vy);
                    double carn_s = sensor_fusion[i][5];

                    /*Predict the other car's future location*/
                    carn_s += ((double)prev_path*0.02*carn_vel);
                    /*Get the closest front car in the right lane and its velocity*/
                    /*Get the slowest car velocity in the right lane*/
                    if (((carn_s > car_s) && ((carn_s-car_s) > NEXT_LANE_FRONT_CAR_DIST_MIN)))
                    {
                      if(right_car_cnt == 0)
                      {
                        min_right_car_vel = carn_vel;
                        right_close_car_dist = carn_s-car_s;
                        right_close_car_vel = carn_vel;
                        right_car_cnt++;
                      }
                      else
                      {
                        if(carn_vel < min_right_car_vel)
                          min_right_car_vel = carn_vel;
                        if((carn_s-car_s) < right_close_car_dist)
                        {
                          right_close_car_dist = carn_s-car_s;
                          right_close_car_vel = carn_vel;
                        }
                        
                      }
                    }

                     /*Check if the front and back cars in the right lane are at safe distance away from self car in the future*/
                    if (((carn_s > car_s) && ((carn_s-car_s) < NEXT_LANE_FRONT_CAR_DIST_MIN)) || ((carn_s < car_s) && ((car_s-carn_s) < NEXT_LANE_BACK_CAR_DIST_MIN)))
                    {
                      right_too_close = true;
                    }
                  }
                }
                if(right_too_close == false)
                {
                  if ((lane == 1) &&  (lane != next_lane))
                    next_lane = 4; // This next_lane is set to 4 to identify that both lane changes are possible
                  else
                    next_lane = lane+lane_delta;
                }
              }
              /*If next lane is same current lane then lane change is not possible*/
              if(next_lane == lane)
              {
                cout << "1.Lane change not possible: Front " << too_close << " Left " << left_too_close << " Right " << right_too_close << endl;
                ref_vel -= DECELERATE;
              }
              /*If lane change is possible for both left lane & right lane when ego car in the middle lane*/
              else if (next_lane == 4)
              {
                /*If no cars ahead in the left lane then take left lane*/
                if (left_car_cnt == 0)
                {
                  lane = 0;
                }
                /*If cars are present in both lanes then do the lane change to the fastest lane*/
                else if (left_car_cnt > 0 && right_car_cnt > 0)
                {
                  bool left_lane_change = false;
                  bool right_lane_change = false;
                  
                  /*If relative velocity of the left car in w.r.t front car is positive OR if left car is far off then left lane change is possible*/
                  /*If the relative velocity is negative, then it is not beneficial to make left lane change unless the left front car is far off-This might give change for 2 lane changes*/
                  if ((left_close_car_vel > front_car_vel) || (left_close_car_dist > NEXT_LANE_FRONT_CAR_DIST_MAX))
                  {
                    left_lane_change = true;
                  }

                  /*If relative velocity of the right car in w.r.t front car is positive OR if left car is far off then right lane change is possible*/
                  /*If the relative velocity is negative, then it is not beneficial to make right lane change unless the left front car is far off-This might give change for 2 lane changes*/
                  if ((right_close_car_vel > front_car_vel) || (right_close_car_dist > NEXT_LANE_FRONT_CAR_DIST_MAX))
                  {
                    right_lane_change = true;
                  }
                  
                  /*If both the lane changes are possible, then select the fastest lane based on the slowest car velocity in either*/
                  /*This will ensure that we take the faster lane*/
                  if ((left_lane_change == true) && (right_lane_change == true))
                  {
                    if (min_right_car_vel > min_left_car_vel)
                      lane = 2;
                    else
                      lane = 0;
                  }
                  /*If only left lane change is possbile then make the left lane change*/
                  else if ((left_lane_change == true) && (right_lane_change == false))
                  {
                    lane = 0;
                  }
                  /*If only right lane change is possbile then make the right lane change*/
                  else if ((left_lane_change == false) && (right_lane_change == true))
                  {
                    lane = 2;
                  }
                  /*If no lane change is possible then decelerate*/
                  else
                  {
                     cout << "2.Lane change not possible: Front " << too_close << " Left rel velocity " << (left_close_car_vel - front_car_vel) << " Left dist " << left_close_car_dist <<  " Rigth rel velocity " << (right_close_car_vel - front_car_vel) << " Right dist " << right_close_car_dist << endl;
                    ref_vel -= DECELERATE;
                  }
                }
                /*If no cars are present in the right lane and cars are present in the left lane, then do lane change to right lane*/
                else if (left_car_cnt > 0 && right_car_cnt == 0)
                {
                    lane = 2;
                }
              }
              /*If not in the middle lane, then next lane is not equal to current then make the lane change to the next lane*/
              else
              {
                /*Left lane change is only possible*/
                if (next_lane < next_lane)
                {
                  if (left_car_cnt == 0)
                    lane = next_lane;
                  /*If relative velocity of the left car in w.r.t front car is positive OR if left car is far off then left lane change is possible*/
                  /*If the relative velocity is negative, then it is not beneficial to make left lane change unless the left front car is far off-This might give change for 2 lane changes*/
                  else if ((left_close_car_vel > front_car_vel) || (left_close_car_dist > NEXT_LANE_FRONT_CAR_DIST_MAX))
                    lane = next_lane;
                  /*If left lane change is not possible then decelerate*/
                  else
                  {
                    cout << "3.Lane change not possible: Front " << too_close << " Left rel velocity " << (left_close_car_vel - front_car_vel) << " Left dist " << left_close_car_dist << endl;
                    ref_vel -= DECELERATE;
                  }
                }
                /*Right lane change is only possible*/
                else
                {
                  if (right_car_cnt == 0)
                    lane = next_lane;
                  /*If relative velocity of the right car in w.r.t front car is positive OR if left car is far off then right lane change is possible*/
                  /*If the relative velocity is negative, then it is not beneficial to make right lane change unless the left front car is far off-This might give change for 2 lane changes*/
                  else if ((right_close_car_vel > front_car_vel) || (right_close_car_dist > NEXT_LANE_FRONT_CAR_DIST_MAX))
                    lane = next_lane;
                  /*If right lane change is not possible then decelerate*/
                  else
                  {
                    cout << "4.Lane change not possible: Front " << too_close << " Right rel velocity " << (right_close_car_vel - front_car_vel) << " Right dist " << right_close_car_dist << endl;
                    ref_vel -= DECELERATE;
                  }
                }
              }
            }
            /*If the front car is at safe distance or there is not front car then accelerate*/
            else if (ref_vel < MAX_SPEED_LIMIT)
            {
              cout << "5.Accelerate " << endl;
              ref_vel += ACCELERATE;
            }

            /*Create a list of widely spaced points before using spline*/
            vector<double> ptsx;
            vector<double> ptsy;

            /*reference x,y & yaw states*/
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            /*if previous path is empty, use the car as starting reference*/
            if(prev_path < 2)
            {
              /*use two points, one the current car coordinates*/
              /*second the previous point by interpolating based on the current car yaw angle*/
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              /*store the two set of points*/
              ptsx.push_back(prev_car_x);
              ptsy.push_back(prev_car_y);

              ptsx.push_back(car_x);
              ptsy.push_back(car_y);
            }
            /*use the previous path 2 endpoints and calucalte the car yaw pertaining to the last unused waypoint*/
            else
            {
              /*Update the ref_x & ref_y & ref_yaw*/
              ref_x = previous_path_x[prev_path - 1];
              ref_y = previous_path_y[prev_path - 1];

              double ref_x_prev = previous_path_x[prev_path - 2];
              double ref_y_prev = previous_path_y[prev_path - 2];
              
              ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

              ptsx.push_back(ref_x_prev);
              ptsy.push_back(ref_y_prev);

              ptsx.push_back(ref_x);
              ptsy.push_back(ref_y);              

            }

            /*In Frenet space, get 3 evenly spaces points (around 30m apart) w.r.t to current (s,d)*/
            vector<double> next_0_pts = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_1_pts = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_2_pts = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            /*Add these points to the list*/
            ptsx.push_back(next_0_pts[0]);
            ptsy.push_back(next_0_pts[1]);

            ptsx.push_back(next_1_pts[0]);
            ptsy.push_back(next_1_pts[1]);

            ptsx.push_back(next_2_pts[0]);
            ptsy.push_back(next_2_pts[1]);

            /*convert from global coordinates to car coordinates to simplify the calculation*/
            /*The origin of the car coordinates is the last unused waypoint and the car yaw at that position*/
            for (int i =0; i<ptsx.size();i++)
            {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
              ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
              ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
            }

            /*create spline*/
            tk::spline s;
            s.set_points(ptsx,ptsy);

                  

            json msgJson;

            /*Actual way points to be fed to the simulator*/
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            /*Start with the ununsed waypoints of the previous path*/
            for(int i = 0; i<prev_path; i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            /*Parameters to breackup the spline into correct points so that car travels at reference velocity*/
            /*These points will be visited by the car every 20 ms.*/
            /*So the points should be chosen such that the reference velocity is maintained*/
            /*Select origin & the point 30 meteres ahead in x-coordinates on the spline*/
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

            double x_add_on = 0;

            /*Assuming the car goes with a constant velocity on the direct path between these 2 points, select equidistant points on this direct path.*/
            /*Project these points directly on the spline and use the necessary points required for the new trajectory.*/
            for (int i=1; i<=(50 - prev_path); i++)
            {
              /*Get number of points to be used on the spline*/
              double N = (target_dist/(0.02*ref_vel/2.24));
              double x_point = x_add_on + (target_x)/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              /*convert back from car coorinated to global coordinates*/
              x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
              y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
