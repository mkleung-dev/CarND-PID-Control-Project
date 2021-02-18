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

  int timeToOptimize = 0;
  double steer_Kp = 0.4;
  double steer_Ki = 0.00001;
  double steer_Kd = 6.0;
  double target_speed = 20;
  RunningRMSE rmse;
  if (argc >= 3) {
    steer_Kp = atof(argv[1]);
    steer_Ki = atof(argv[2]);
    steer_Kd = atof(argv[3]);
  }
  /**
   * Initialize the pid variable.
   */
  // double Kp_, double Ki_, double Kd_
  steer_pid.Init(steer_Kp, steer_Ki, steer_Kd);
  velocity_pid.Init(0.1, 0.002, 0.0);
  rmse.Reset();

  h.onMessage([&target_speed, &rmse, &timeToOptimize, &steer_pid, &velocity_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          if (0 >= timeToOptimize && steer_pid.IsOptimizationFinished()) {
            steer_pid.startOptimization(10, 0.001, {0.01, 0.000001, 0.1});
            timeToOptimize = 10;
          }
          steer_pid.UpdateError(cte);
          rmse.AddError(cte);
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

          if (steer_pid.IsOptimizationFinished()) {
            timeToOptimize--;
          }
          if (rmse.GetCount() > 3000) {
            if (rmse.getRMSE() < 0.01) {
              target_speed += 10;
            }
            rmse.Reset();
          }

          std::cout << "cte," << cte << std::endl;
          std::cout << "speed," << speed << std::endl;
          std::cout << "rmse.getRMSE()," << rmse.getRMSE() << std::endl;
          std::cout << "rmse.GetCount()," << rmse.GetCount() << std::endl;
          std::cout << "target_speed," << target_speed << std::endl;
          std::cout << "angle," << angle << std::endl;
          std::cout << "steer_value," << steer_value << std::endl;
          // DEBUG
          std::cout << "Kp: " << steer_pid.get_Kp() << " Ki: " << steer_pid.get_Ki() << " Kd: " << steer_pid.get_Kd() << std::endl;


          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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