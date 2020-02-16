#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main()
{
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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
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

  // Start in lane 1: middle lane (left is 0 and right is 2)
  int current_lane = 1;

  // Reference velocity: start with 0.0
  double ref_vel = 0.0; // mph

  h.onMessage([&ref_vel, &current_lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
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

          // Sensor Fusion Data, a list of all other cars on the same side
          //  of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // Provided previous path point size.
          int prev_size = previous_path_x.size();

          // Avoid collisions with front cars
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }

          // Analysis sensor fusion list: it contains all information on other cars;
          // d, s, xy,vx and vy values for each car
          bool car_ahead = false;
          bool car_left = false;
          bool car_right = false;
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            float car_i_d = sensor_fusion[i][6];
            int car_i_lane = -1;

            // Identify in which lane the car_i is
            // Left most lane: 0
            if (car_i_d > 0 && car_i_d < 4)
            {
              car_i_lane = 0;
            }
            // Center lane: 1
            else if (car_i_d > 4 && car_i_d < 8)
            {
              car_i_lane = 1;
            }
            // Right most lane: 2
            else if (car_i_d > 8 && car_i_d < 12)
            {
              car_i_lane = 2;
            }
            // If error in detecting the lane, put current_lane as default value
            else
            {
              car_i_lane = current_lane;
            }

            // Car_i_speed values
            double car_i_vx = sensor_fusion[i][3];
            double car_i_vy = sensor_fusion[i][4];

            // Car_i velocity
            double check_speed = sqrt(car_i_vx * car_i_vx + car_i_vy * car_i_vy);

            // Car_i s in Frenet coordinates
            double check_car_i_s = sensor_fusion[i][5];

            // Estimate car_i s position projecting previous path
            check_car_i_s += ((double)prev_size * 0.02 * check_speed);
            if (car_i_lane == current_lane)
            {
              // Car in our lane.
              car_ahead |= check_car_i_s > car_s && check_car_i_s - car_s < 30;
            }
            else if (car_i_lane - current_lane == -1)
            {
              // Car left
              car_left |= car_s - 30 < check_car_i_s && car_s + 30 > check_car_i_s;
            }
            else if (car_i_lane - current_lane == 1)
            {
              // Car right
              car_right |= car_s - 30 < check_car_i_s && car_s + 30 > check_car_i_s;
            }
          }

          // Behavior : Let's see what to do.
          double behavior_velocity = 0;

          if (car_ahead)
          { // Car ahead
            if (!car_left && current_lane > 0)
            {
              // If there is no car left and there is a left lane.
              current_lane--;
            }
            else if (!car_right && current_lane != 2)
            {
              // If there is no car right and there is a right lane.
              current_lane++;
            }
            else
            {
              // Slow down the vehicle because all lanes are occupied and there is slower car in front of us
              behavior_velocity -= .224; //
            }
          }
          else
          {
            if (current_lane != 1)
            {
              // If we are not on the center lane and there are no car in the center lane
              if ((current_lane == 0 && !car_right) || (current_lane == 2 && !car_left))
              {
                // Goes to center lane
                current_lane = 1;
              }
            }
            if (ref_vel < 49.5)
            {
              // If there are no car_ahead and the ref_vel is not greater than the maximum
              behavior_velocity += .224;
            }
          }

          // List if spaced waypoints
          // Spline interpolation
          vector<double> ptsx;
          vector<double> ptsy;

          // Reference x, y, yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Check if previous state is empyt:
          if (prev_size < 2)
          {
            // Use car data (x, y) as starting reference
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // If previous path is not empty
          else
          {
            // Use the last two points as reference.
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // Use previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // Setting up target points in the future.
          // In Frenet's coordinate: 30m spaced points ahead

          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * current_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * current_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * current_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // Making coordinates to local car coordinates.
          // Previous coordinates should be (x = 0,y = 0 ,yaw_rate = 0)
          for (int i = 0; i < ptsx.size(); i++)
          {
            // Shift and rotation
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // Create the spline.
          tk::spline s;

          // Set point x any to the spline
          s.set_points(ptsx, ptsy);

          // Define actual (x, y) used in the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start from last previous path's points
          for (int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate target (x, y) position: x is 30 m ahead.
          double target_x = 30.0;                                               // horizon
          double target_y = s(target_x);                                        //target y for the given x
          double target_dist = sqrt(target_x * target_x + target_y * target_y); //euclidean disatnce

          // starting point where to add x
          double x_add_on = 0;

          for (int i = 1; i < 50 - prev_size; i++)
          {
            // As suggested in the walkthrough, add velocity here for efficiency
            ref_vel += behavior_velocity;
            if (ref_vel > 49.5)
            {
              // If the velocity is breaking the speed limit, put back to a accetable value
              ref_vel = 49.5;
            }
            // else if (ref_vel < .224)
            // {
            //   ref_vel += .224;
            // }

            double N = target_dist / (0.02 * ref_vel / 2.24); //2.24 conversion into meters per second
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Go back to global coordinates: shift and rotation
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            // Attach new points
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}