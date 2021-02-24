#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "PIDWithTwiddle.h"

#include "RunningRMSE.h"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char *argv[]) {
  uWS::Hub h;

  PIDWithTwiddle steer_pid;
  PID velocity_pid;

  bool timeToOptimize = true;

// 40
// ,Kp,0.16957,Ki,9.33996e-07,Kd,1.32288
// ,dp0,0.00117269,dp1,9.55522e-09,dp2,0.0143328
// ,dpFinished0,0.0015,dpFinished1,1e-08,dpFinished2,0.015

// 45
// ,Kp,0.182299,Ki,7.39706e-07,Kd,1.36296
// ,dp0,0.00131243,dp1,8.83529e-09,dp2,0.00930794
// ,dpFinished0,0.0016957,dpFinished1,9.33996e-09,dpFinished2,0.0132288

// 50
// ,Kp,0.169904,Ki,7.39706e-07,Kd,1.36296
// ,dp0,0.000333975,dp1,1.45688e-09,dp2,0.00268441
// ,dpFinished0,0.00033914,dpFinished1,1.47941e-09,dpFinished2,0.00272592

// 60
// ,Kp,0.0778486,Ki,7.64535e-07,Kd,1.29949
// ,dp0,0.000121371,dp1,1.37163e-09,dp2,0.00120026
// ,dpFinished0,0.00016,dpFinished1,1.47941e-09,dpFinished2,0.0026


  // double steer_Kp = 0.3;
  // double steer_Ki = 0.0000001;
  // double steer_Kd = 0.2;
  // double target_speed = 10;
  // double steer_Kp = 0.25;
  // double steer_Ki = 0.0000001;
  // double steer_Kd = 1.0;
  // double target_speed = 20;
  // double steer_Kp = 0.2;
  // double steer_Ki = 0.0000001;
  // double steer_Kd = 1.2;
  // double target_speed = 30;
  // double steer_Kp = 0.2;
  // double steer_Ki = 0.0000001;
  // double steer_Kd = 1.2;
  // double target_speed = 40;
  // double steer_Kp = 0.15;
  // double steer_Ki = 0.0000001;
  // double steer_Kd = 1.2;
  // double target_speed = 50;

  double steer_Kp_high_speed = 0.08;
  double steer_Ki_high_speed = 0.0000001;
  double steer_Kd_high_speed = 0.2;
  double target_high_speed = 70;

  double steer_Kp_low_speed = 0.15;
  double steer_Ki_low_speed = 0.00001;
  double steer_Kd_low_speed = 1.8;
  double target_low_speed = 30;

  RunningRMSE rmse(10);
  if (argc >= 4) {
    steer_Kp_high_speed = atof(argv[1]);
    steer_Ki_high_speed = atof(argv[2]);
    steer_Kd_high_speed = atof(argv[3]);
  }
  if (argc >= 5) {
    target_high_speed = atof(argv[4]);
  }
  /**
   * Initialize the pid variable.
   */
  // double Kp_, double Ki_, double Kd_
  steer_pid.Init(steer_Kp_high_speed, steer_Ki_high_speed, steer_Kd_high_speed);
  velocity_pid.Init(0.3, 0.00001, 0.4);

  h.onMessage([&steer_Kp_high_speed, &steer_Ki_high_speed, &steer_Kd_high_speed, &target_high_speed,
               &steer_Kp_low_speed, &steer_Ki_low_speed, &steer_Kd_low_speed, &target_low_speed,
               &rmse, &timeToOptimize, &steer_pid, &velocity_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value = 0;
          double throttle = 0;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           */
          double target_speed = target_high_speed;
          if (rmse.getRMSE() > 0.8) {
            steer_pid.Init(steer_Kp_low_speed, steer_Ki_low_speed, steer_Kd_low_speed);
            target_speed = target_low_speed;
          } else {
            steer_pid.Init(steer_Kp_high_speed, steer_Ki_high_speed, steer_Kd_high_speed);
            target_speed = target_high_speed;
          }
          rmse.AddError(cte);
          if (timeToOptimize && steer_pid.IsOptimizationFinished() && speed > target_high_speed * 0.9 && false) {
            steer_pid.startOptimization(1000, 0.1, {steer_Kp_high_speed / 50.0, steer_Ki_high_speed / 50.0, steer_Kd_high_speed / 50.0});
            timeToOptimize = false;
          }
          steer_pid.UpdateError(cte);
          steer_value = -steer_pid.TotalError();
          if (steer_value < -1) {
            steer_value = -1;
          }
          if (steer_value > 1) {
            steer_value = 1;
          }

          velocity_pid.UpdateError(speed - target_speed);
          throttle = -velocity_pid.TotalError();
          if (throttle < -1) {
            throttle = -1;
          }
          if (throttle > 1) {
            throttle = 1;
          }

          std::cout << "rmse," << rmse.getRMSE();
          std::cout << "cte," << cte;
          std::cout << ",steer_value," << steer_value;
          std::cout << ",angle," << angle;
          std::cout << ",speed," << speed;
          std::cout << ",target_speed," << target_high_speed;
          std::cout << ",throttle," << throttle;
          std::cout << std::endl;
          std::cout << ",Kp," << steer_pid.get_Kp();
          std::cout << ",Ki," << steer_pid.get_Ki();
          std::cout << ",Kd," << steer_pid.get_Kd();
          std::cout << std::endl;
          std::cout << ",dp0," << steer_pid.getDp(0);
          std::cout << ",dp1," << steer_pid.getDp(1);
          std::cout << ",dp2," << steer_pid.getDp(2);
          std::cout << std::endl;
          std::cout << ",dpFinished0," << steer_pid.getDpFinished(0);
          std::cout << ",dpFinished1," << steer_pid.getDpFinished(1);
          std::cout << ",dpFinished2," << steer_pid.getDpFinished(2);
          std::cout << std::endl;
          std::cout << ",getOptimizationIndex," << steer_pid.getOptimizationIndex();
          std::cout << ",getState," << (int)steer_pid.getState();
          std::cout << ",getRMSECount," << steer_pid.getRMSECount();
          std::cout << ",TwiddleFinished," << (steer_pid.IsOptimizationFinished() ? "Yes" : "No");
          std::cout << ",timeToOptimize," << (timeToOptimize ? "Yes" : "No");
          std::cout << std::endl;


          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}