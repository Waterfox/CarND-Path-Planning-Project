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
  double disc_inc = 0.0;
  double acc = 0;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &disc_inc, &acc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
            int path_N = 40; // target path size
            //load previous path into next path
            //This will be approximately N - 2 points (2 to 3 points get consumed per iteration)
            for(int i = 0; i < path_size; i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            if(path_size < 2)
            {
                pos_x = car_x;
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

            //Use the current car position to find the next waypoint. Fit a spline to car position and next 4 waypoints in global xy
            int next_wp = NextWaypoint(car_x,car_y,car_yaw,map_waypoints_x,map_waypoints_y);
            int prev_wp = next_wp - 1;
            cout << next_wp << endl;
            if(next_wp == 0)
            {
              prev_wp  = map_waypoints_x.size()-1;
            }
            int n_spl_pts = 5;
            std::vector<double> spl_ptsx;
            std::vector<double> spl_ptsy;
            std::vector<double> spl_ptss;
            // spl_ptsx.push_back(map_waypoints_x[prev_wp]);
            // spl_ptsy.push_back(map_waypoints_y[prev_wp]);
            // spl_ptss.push_back(map_waypoints_s[prev_wp]);

            //add the last enpoint from the previous path
            spl_ptsx.push_back(pos_x2);
            spl_ptsy.push_back(pos_y2);
            spl_ptsx.push_back(pos_x);
            spl_ptsy.push_back(pos_y);
            // spl_ptss.push_back(pos_s);
            cout << "["<<map_waypoints_x[(next_wp+0) % map_waypoints_x.size()] <<", "<< map_waypoints_y[(next_wp+0) % map_waypoints_y.size()]<<"], ";

            for (int i=2; i<n_spl_pts; i++){
              spl_ptsx.push_back(map_waypoints_x[(next_wp+i) % map_waypoints_x.size()]);
              spl_ptsy.push_back(map_waypoints_y[(next_wp+i) % map_waypoints_y.size()]);
              // spl_ptss.push_back(map_waypoints_s[(next_wp+i) % map_waypoints_s.size()]);
              cout << "["<<map_waypoints_x[(next_wp+i) % map_waypoints_x.size()] <<", "<< map_waypoints_y[(next_wp+i) % map_waypoints_y.size()]<<"], ";
              // cout << map_waypoints_y[(next_wp+i) % map_waypoints_y.size()] <<", ";
            }
            cout<<endl;

            //Transform to car coordinates
            std::vector<double> spl_ptsx_c(spl_ptsx.size());
            std::vector<double> spl_ptsy_c(spl_ptsy.size());
            double yaw_rad = deg2rad(angle);
            // double prev_x = pos_x;
            for (int i=0; i<spl_ptsx.size();i++ ){
              spl_ptsx_c[i] = (spl_ptsx[i] - pos_x) * cos(0-yaw_rad) - (spl_ptsy[i] - pos_y) * sin(0-yaw_rad);
              spl_ptsy_c[i] = (spl_ptsx[i] - pos_x) * sin(0-yaw_rad) + (spl_ptsy[i] - pos_y) * cos(0-yaw_rad);
              cout << "["<<spl_ptsx_c[i] <<", "<<spl_ptsy_c[i]<<"], ";
              // cout << spl_ptsy_c[i] <<", ";
            }
            cout<<endl;


            tk::spline spl1;
            spl1.set_points(spl_ptsx_c,spl_ptsy_c);


            // int n_eval_pts = 100;
            // std::vector<double> local_x(n_eval_pts);
            // std::vector<double> local_y(n_eval_pts);
            // std::vector<double> local_s(n_eval_pts);
            // std::vector<double> local_xy(2);
            //
            // for (int i=0;i<n_eval_pts;i++){
            //   double s = car_s+i*0.4;
            //   local_s[i] = s;
            //   cout << s <<endl;
            //   local_xy = getXY(s, 0, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            //   local_x[i] = local_xy[0];
            //   local_y[i] = local_xy[1]; //spl1(local_xy[0]);
            // }



            // manage accelerations
            double target_velocity_mph = 47.0; //mph
            double target_velocity = target_velocity_mph * 0.44704; //m/s
            double dt = 0.02;// s
            double a_max = 10.0; //m/s^2
            double j_max = 100.0;

            double car_speed_ms = car_speed * 044704; // m/s

            //calculate acceleration based on max jerk
            if (target_velocity_mph > car_speed){
              //disc_inc = car_speed_ms* dt + 0.5*a_max*dt*dt;
              //accelerate
              if (acc < a_max){acc = acc + j_max*dt;}


            }
            else if (target_velocity_mph < car_speed){
              //disc_inc = car_speed_ms* dt - 0.5*a_max*dt*dt;
              //deccelerate
              if (acc > -a_max) {acc = acc - j_max*dt;}
            }
            else {
              //zero accelleration
              if (acc>0.0) { acc = acc - j_max*dt;}
              else if (acc < 0.0) {acc = acc + j_max*dt;}
              else {acc = acc;} //acc is zero
            }

            disc_inc = disc_inc + 0.5*acc*dt*dt; // screw managing sign of jerk + 1.0/6.0*j_max*dt*dt*dt;

            // // Just use acceleration
            // if (target_velocity_mph > car_speed){
            //   //disc_inc = car_speed_ms* dt + 0.5*a_max*dt*dt;
            //   disc_inc = disc_inc + 0.5*a_max*dt*dt;
            // }
            // else if (target_velocity_mph < car_speed){
            //   //disc_inc = car_speed_ms* dt - 0.5*a_max*dt*dt;
            //   disc_inc = disc_inc - 0.5*a_max*dt*dt;
            // }
            // else {
            //   disc_inc = disc_inc;
            // }
            cout << disc_inc <<endl;



            //cout << path_size << '\n';
            //fill the remaining path up to 50
            for(int i = 0; i < path_N-path_size; i++)
            {
              // calc next s,d
              pos_s += disc_inc;
              pos_d = 6;

              // convert s,d to x,y
              vector<double> pos_xy = getXY(pos_s, pos_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              pos_x = pos_xy[0];
              pos_y = pos_xy[1];
              //cout << pos_s " << i << endl;
              // add these to the next path

              next_x_vals.push_back(pos_x);
              next_y_vals.push_back(pos_y);
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
