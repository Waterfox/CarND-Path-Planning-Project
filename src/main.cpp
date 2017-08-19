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
#define MPH_TO_MS  0.44704;

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
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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

// vector<double> getXY2(double s, double d, double car_x, double car_y, double car_s, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y){
//   //This one fits a spline to the current possition and 4 of the global wpts
//
//
// }

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
  double vel_out = 0.0;
  double last_vel_out = 0.0;
  double acc = 0;
  int lane = 0;
  double set_vel = 49.0*MPH_TO_MS;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &vel_out, &last_vel_out, &set_vel, &lane, &acc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds



            double pos_x;
            double pos_x2;
            double pos_y;
            double pos_y2;
            double angle;
            double pos_s;
            double pos_d;

            int path_size = previous_path_x.size(); // returned path size
            int path_N = 50; // target path size

            //load previous path into next path
            //This will be approximately N - 2 points (2 to 3 points get consumed per iteration)
            for(int i = 0; i < path_size; i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            if(path_size < 2)
            {
                pos_x = car_x; //last x position
                pos_y = car_y;
                angle = deg2rad(car_yaw);
                pos_x2 = car_x - cos(angle);
                pos_y2 = car_y - sin(angle);
                pos_s = car_s;
                pos_d = car_d;
            }
            else
            {
                //last element of the path
                pos_x = previous_path_x[path_size-1];
                pos_y = previous_path_y[path_size-1];

                pos_x2 = previous_path_x[path_size-2];
                pos_y2 = previous_path_y[path_size-2];
                angle = atan2(pos_y-pos_y2,pos_x-pos_x2);

                //vector<double> pos_sd = getFrenet(pos_x,pos_y,angle,map_waypoints_x,map_waypoints_y);
                //pos_s = pos_sd[0];
                //pos_d = pos_sd[1];
                pos_s = end_path_s;
                pos_d = end_path_d;
            }

            cout << "car_s: " << car_s << endl;
            cout << "car_d: " << car_d << endl;

            // spl_ptss.push_back(pos_s);


            //--generate waypoints along centerline based on map_waypoints--
            // //Use the current car position to find the next waypoint. Fit a spline to car position and next 4 waypoints in global xy
            // int next_wp = NextWaypoint(car_x,car_y,car_yaw,map_waypoints_x,map_waypoints_y);
            // int prev_wp = next_wp - 1;
            // cout << next_wp << endl;
            // if(next_wp == 0)
            // {
            //   prev_wp  = map_waypoints_x.size()-1;
            // }
            //cout << "["<<map_waypoints_x[(next_wp+0) % map_waypoints_x.size()] <<", "<< map_waypoints_y[(next_wp+0) % map_waypoints_y.size()]<<"], ";
            // spl_ptsx.push_back(map_waypoints_x[prev_wp]);
            // spl_ptsy.push_back(map_waypoints_y[prev_wp]);
            // for (int i=2; i<n_spl_pts; i++){
            //   spl_ptsx.push_back(map_waypoints_x[(next_wp+i) % map_waypoints_x.size()]);
            //   spl_ptsy.push_back(map_waypoints_y[(next_wp+i) % map_waypoints_y.size()]);
            //   // spl_ptss.push_back(map_waypoints_s[(next_wp+i) % map_waypoints_s.size()]);
            //   cout << "["<<map_waypoints_x[(next_wp+i) % map_waypoints_x.size()] <<", "<< map_waypoints_y[(next_wp+i) % map_waypoints_y.size()]<<"], ";
            //   // cout << map_waypoints_y[(next_wp+i) % map_waypoints_y.size()] <<", ";
            // }
            //cout<<endl;

            //--generate waypoints in front of the car based on getXY

            //*********------------BEHAVIOR-----******

            //sample behavior

            // if (car_s > 200.0){lane = 1.0; target_vel = 35.0*MPH_TO_MS;}
            // if (car_s > 350.0){lane = 2.0; target_vel = 20.0*MPH_TO_MS;}
            // if (car_s > 500.0){lane = 1.0; target_vel = 47.0*MPH_TO_MS;}
            // if (car_s > 650.0){lane = 0.0; target_vel = 40.0*MPH_TO_MS;}

            string state = "KEEPLANE";
            //review telemetry;
            double min_tar_dist = 999999;
            double min_tar_vel = set_vel;
            double min_tar_idx;

            for (int i = 0; i < sensor_fusion.size(); i++){

              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double tar_vel = sqrt(pow(vx,2)+pow(vy,2));
              double tar_s = sensor_fusion[i][5];
              double tar_d = sensor_fusion[i][6];


              if (tar_d < (2+4*lane+2) && tar_d >(2+4*lane-2) ) //check if car in my lane - from project walkthrough
              {
                double ds = tar_s - car_s; //delta_s coord
                //cout << "in_lane: " << i << " v: "<< tar_vel << " ds: "<<ds <<  endl;

                if (ds < min_tar_dist && ds > 0.0) // who is the closest car in front of me. //TODO Handle WRAP AROUND
                {
                  min_tar_dist = ds;
                  min_tar_idx = i;
                  min_tar_vel = tar_vel;
                }
              }
            }
            double follow_dist = 25.0; // match the vehicle's speed in front of you, consider a lane change
            double safety_dist = 18.0; // the vehicle will slow down by 1m/s rel to the car in front of it in this distance. Useful for getting unstuck

            if(min_tar_dist < follow_dist)
            {
              state = "FOLLOW";
              set_vel = min_tar_vel;
              if (min_tar_dist < safety_dist) // if we are getting too close, back off
              {set_vel = min_tar_vel - 1.0;}
            }
            else
            {
              state = "KEEPLANE";
              set_vel = 49.0*MPH_TO_MS;
            }
            cout <<"min_tar_dist: "<<min_tar_dist << endl;

            // See if we should change lanes
            if (state == "FOLLOW")
            {
              // Try to change lanes

              double look_backwards = -3.0; // we are considering cars up to 3 m behind us in the opposite lane when changing lanes
              double min_ds_left = look_backwards-1.0; // this is chosen to be one unit less than the minumum value of ds (delta_s) can be set to
              double min_ds_right = look_backwards-1.0;
              double min_left_vel = 49.0*MPH_TO_MS; // plan the lane change at the velocity of the vehicle in front of you, so you don't hit it.
              double min_right_vel = 49.0*MPH_TO_MS;
              //-------check the left lane-------------
              if (lane != 0)
              {
                int target_lane = lane - 1;
                //review telemetry for cars in this lane
                min_ds_left = 99998; // bias right if both lanes are clear
                for (int i = 0; i < sensor_fusion.size(); i++){

                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double tar_vel = sqrt(pow(vx,2)+pow(vy,2));
                  double tar_s = sensor_fusion[i][5];
                  double tar_d = sensor_fusion[i][6];


                  if (tar_d < (2+4*target_lane+2) && tar_d >(2+4*target_lane-2) ) //check if car in my lane - from project walkthrough
                  {
                    double ds = tar_s - car_s; //delta_s coord - difference of our s coord from other car's s
                    // cout << "in_lane: " << i << " v: "<< tar_vel << " ds: "<<ds <<  endl;

                    if (ds < min_ds_left && ds > look_backwards) // who is the closest car in front of me or the car up to 3 m behind me
                    {
                      min_ds_left = ds;
                      min_left_vel = tar_vel;
                    }
                  }
                }
              }

              //--------check the right lane---------
              if (lane != 2 )
              {
                int target_lane = lane + 1;
                //review telemetry for cars in this lane
                min_ds_right = 99999;
                for (int i = 0; i < sensor_fusion.size(); i++){

                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double tar_vel = sqrt(pow(vx,2)+pow(vy,2));
                  double tar_s = sensor_fusion[i][5];
                  double tar_d = sensor_fusion[i][6];


                  if (tar_d < (2+4*target_lane+2) && tar_d >(2+4*target_lane-2) ) //check if car in my lane - from project walkthrough
                  {
                    double ds = tar_s - car_s; //delta_s coord
                    // cout << "in_lane: " << i << " v: "<< tar_vel << " ds: "<<ds <<  endl;

                    if (ds < min_ds_right && ds > look_backwards) // who is the closest car in front of me. //TODO Handle WRAP AROUND
                    {
                      min_ds_right = ds;
                      min_right_vel = tar_vel;
                    }
                  }
                }
              }


              //------Take an action based on the results of right and left lane check

              //stay in our current lane
              if (min_tar_dist < 10) // don't change lanes if we are too close to a car, slow down first
              {
                state = "FOLLOW";
              }
              // CHANGE RIGHT
              else if (min_ds_right > min_ds_left && min_ds_right > 22.0) // change lanes right if there is more room on the right, make sure there is 22m
              {
                lane = lane + 1;
                state = "KEEPLANE";
                set_vel = min(min_right_vel,min_tar_vel); //our change lane speed is the minimum of the car in the lane beside us and the one we are following in our current lane
              }
              // CHANGE LEFT
              else if (min_ds_left > min_ds_right && min_ds_left > 22.0) // change lanes left if there is more room on the left
              {
                lane = lane -1;
                state = "KEEPLANE";
                set_vel = min(min_left_vel,min_tar_vel); //our change lane speed is the minimum of the car in the lane beside us and the one we are following in our current lane
              }

              // KEEP LANE
              else // otherwise keep following //TODO confirm how much we need this
              {
                state = "FOLLOW";
                // set_vel = 49.0*MPH_TO_MS
              }

              cout << "min_ds_left:" << min_ds_left << " min_ds_right: " << min_ds_right << endl;
            }
            cout << "State: " << state << " lane: "<< lane <<endl;

            //TODO: Stop double lane changes, they JERK --



            //------------Fit a Spline-------------------------------
            // the spline will be 5 points
            int n_spl_pts = 5;
            std::vector<double> spl_ptsx;
            std::vector<double> spl_ptsy;

            //ensure tangency to current path:
            //add the second last endpoint from the previous path
            spl_ptsx.push_back(pos_x2); //PT1
            spl_ptsy.push_back(pos_y2);
            //add the last endpoint from the previous path
            spl_ptsx.push_back(pos_x);  // PT2
            spl_ptsy.push_back(pos_y);

            //add 3 more points further along the road using s,d coords
            vector<double> wp0 = getXY(car_s+50.0,(2.0+4.0*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            vector<double> wp1 = getXY(car_s+75.0,(2.0+4.0*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            vector<double> wp2 = getXY(car_s+100.0,(2.0+4.0*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            spl_ptsx.push_back(wp0[0]);
            spl_ptsy.push_back(wp0[1]);
            spl_ptsx.push_back(wp1[0]);
            spl_ptsy.push_back(wp1[1]);
            spl_ptsx.push_back(wp2[0]);
            spl_ptsy.push_back(wp2[1]);

            //Transform the spline points to car coordinates
            std::vector<double> spl_ptsx_c(spl_ptsx.size());
            std::vector<double> spl_ptsy_c(spl_ptsy.size());

            double yaw_rad = angle;

            //shift and rotate points
            for (int i=0; i<spl_ptsx.size();i++ ){
              spl_ptsx_c[i] = (spl_ptsx[i] - pos_x) * cos(0.0-yaw_rad) - (spl_ptsy[i] - pos_y) * sin(0.0-yaw_rad);
              spl_ptsy_c[i] = (spl_ptsx[i] - pos_x) * sin(0.0-yaw_rad) + (spl_ptsy[i] - pos_y) * cos(0.0-yaw_rad);
              //cout << "["<<spl_ptsx_c[i] <<", "<<spl_ptsy_c[i]<<"], ";
              // cout << spl_ptsy_c[i] <<", ";
            }
            //cout<<endl;
            //shift the first spline point in X by a small amount to ensure compatibility with the spline library
            spl_ptsx_c[0] = spl_ptsx_c[0]-0.001;


            //fit a spline
            tk::spline spl1;
            spl1.set_points(spl_ptsx_c,spl_ptsy_c);


            //This is used to find point spacing on the spline to ensure target velocity is maintained.
            double target_x = 30.0; //Arbitrary distance, used to find spline y
            double target_y = spl1(target_x);
            double target_dist = sqrt(pow(target_x,2)+pow(target_y,2));
            double x_add_on = 0.0;
            // cout << "target_y: "<<target_y<<endl;
            // cout << "target_dist: "<<target_dist<<endl;



            //Adjust the velocity based on the max_acceleration
            //
            double dt = 0.02;// s
            double a_max = 10.0; //m/s^2
            double j_max = 100.0;

            if (last_vel_out < set_vel){
            vel_out = last_vel_out + a_max*dt;
            last_vel_out = vel_out;
            }
            else if (last_vel_out > set_vel){
            vel_out = last_vel_out - a_max*dt;
            last_vel_out = vel_out;
            }


            //cout << path_size << '\n';
            //fill the remaining path up to 50
            for(int i = 0; i < path_N-path_size; i++)
            {

              // cout << "target_dist: "<<target_dist<<endl;
              double N = target_dist/(0.02*vel_out); //number of points required to acheive spacing for vel_out over a distance of target_dist
              double x_point = x_add_on+(target_x/N); // the next point along the spline in x
              double y_point = spl1(x_point);
              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              //rotate then shift back to global XY
              x_point = (x_ref*cos(yaw_rad)-y_ref*sin(yaw_rad));// rotate points into the global frame using the car yaw
              y_point = (x_ref*sin(yaw_rad)+y_ref*cos(yaw_rad));
              x_point += pos_x; // shift the point into XY coordinates using the last point on the prev_path
              y_point += pos_y;


              // add these to the next path
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);



            }



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
