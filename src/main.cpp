#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"

#include "PID.h"
#include "PIDWithTwiddle.h"
#include "RunningData.h"

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
  RunningData rmse(10);

  // Some Tuned Parameters
  // Different PC may have different parameters.

  // Speed 30
  // Kp 0.389131
  // Ki 0.0000000121714
  // Kd 2.60246

  // Speed 40
  // Kp 0.16957
  // Ki 9.33996e-07
  // Kd 1.32288

  // Kp,0.130391,Ki,9.8713e-09,Kd,1.30835,err,0.650601
  // Speed 50
  // Kp 0.130391
  // Ki 0.0000000098713
  // Kd 1.30835

  // Speed 60
  // Kp 0.0778486
  // Ki 7.64535e-07
  // Kd 1.29949

  // Speed 70
  // Kp 
  // Ki 
  // Kd 

  // Speed 80
  // Kp None
  // Ki None
  // Kd None

  // Speed 90
  // Kp None
  // Ki None
  // Kd None

  // Speed 100
  // Kp None
  // Ki None
  // Kd None

  bool tuning = false;
  bool dual_speed = true;

  double steer_Kp_high_speed = 0.0841968;
  double steer_Ki_high_speed = 0.0000000079;
  double steer_Kd_high_speed = 1.2;
  double target_high_speed = 65;

  double steer_Kp_low_speed = 0.389131;
  double steer_Ki_low_speed = 0.0000000121714;
  double steer_Kd_low_speed = 2.60246;
  double target_low_speed = 30;

  double steer_KpD = steer_Kp_high_speed / 10;
  double steer_KiD = steer_Ki_high_speed / 10;
  double steer_KdD = steer_Kd_high_speed / 10;

  // Handle argument
  if (argc > 1) {
    if (0 == strncmp(argv[1], "tune", 4)) {
      tuning = true;
      dual_speed = false;
      if (argc > 5) {
        target_high_speed = atof(argv[2]);
        steer_Kp_high_speed = atof(argv[3]);
        steer_Ki_high_speed = atof(argv[4]);
        steer_Kd_high_speed = atof(argv[5]);
      }
      if (argc > 8) {
        steer_KpD = atof(argv[6]);
        steer_KiD = atof(argv[7]);
        steer_KdD = atof(argv[8]);
      } else {
        steer_KpD = steer_Kp_high_speed / 10;
        steer_KiD = steer_Ki_high_speed / 10;
        steer_KdD = steer_Kd_high_speed / 10;
      }
    } else if (0 == strncmp(argv[1], "run", 3)) {
      tuning = false;
      dual_speed = true;
      if (argc > 5) {
        target_high_speed = atof(argv[2]);
        steer_Kp_high_speed = atof(argv[3]);
        steer_Ki_high_speed = atof(argv[4]);
        steer_Kd_high_speed = atof(argv[5]);
      }
      if (argc > 9) {
        target_low_speed = atof(argv[6]);
        steer_Kp_low_speed = atof(argv[7]);
        steer_Ki_low_speed = atof(argv[8]);
        steer_Kd_low_speed = atof(argv[9]);
      } else {
        dual_speed = false;
      }
    }
  }

  // Print the running parameters
  if (tuning) {
    std::cout << "Mode:Tuning using Twddle" << std::endl;
    std::cout << "Speed: " << target_high_speed << std::endl;
    std::cout << "Kp: " << steer_Kp_high_speed << std::endl;
    std::cout << "Ki: " << steer_Ki_high_speed << std::endl;
    std::cout << "Kd: " << steer_Kd_high_speed << std::endl;
    std::cout << "Kp Initial Step: " << steer_KpD << std::endl;
    std::cout << "Ki Initial Step: " << steer_KiD << std::endl;
    std::cout << "Kd Initial Step: " << steer_KdD << std::endl;
  } else {
    if (dual_speed) {
      std::cout << "Mode:Running with 2 speeds" << std::endl;
      std::cout << "High Speed: " << target_high_speed << std::endl;
      std::cout << "High Speed Kp: " << steer_Kp_high_speed << std::endl;
      std::cout << "High Speed Ki: " << steer_Ki_high_speed << std::endl;
      std::cout << "High Speed Kd: " << steer_Kd_high_speed << std::endl;
      std::cout << "Low Speed: " << target_low_speed << std::endl;
      std::cout << "Low Speed Kp: " << steer_Kp_low_speed << std::endl;
      std::cout << "Low Speed Ki: " << steer_Ki_low_speed << std::endl;
      std::cout << "Low Speed Kd: " << steer_Kd_low_speed << std::endl;
    } else {
      std::cout << "Mode:Running" << std::endl;
      std::cout << "Speed: " << target_high_speed << std::endl;
      std::cout << "Kp: " << steer_Kp_high_speed << std::endl;
      std::cout << "Ki: " << steer_Ki_high_speed << std::endl;
      std::cout << "Kd: " << steer_Kd_high_speed << std::endl;
    }

  }
  /**
   * Initialize the pid variable.
   */
  steer_pid.Init(steer_Kp_high_speed, steer_Ki_high_speed, steer_Kd_high_speed);
  velocity_pid.Init(0.3, 0.00001, 0.4);
  int frame = 0;

  h.onMessage([&frame, &tuning, &dual_speed,
               &target_high_speed, &steer_Kp_high_speed, &steer_Ki_high_speed, &steer_Kd_high_speed, 
               &target_low_speed, &steer_Kp_low_speed, &steer_Ki_low_speed, &steer_Kd_low_speed,
               &steer_KpD, &steer_KiD, &steer_KdD,
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
          
          // Calculating Steering Angle using PID
          double target_speed = target_high_speed;
          // Handle 2 Speed Mode
          if (dual_speed) {
            if (rmse.getRMS() > 0.8) {
              steer_pid.Init(steer_Kp_low_speed, steer_Ki_low_speed, steer_Kd_low_speed);
              target_speed = target_low_speed;
            } else {
              steer_pid.Init(steer_Kp_high_speed, steer_Ki_high_speed, steer_Kd_high_speed);
              target_speed = target_high_speed;
            }
          }
          rmse.Add(cte);
          // Handle Auto Tuning Mode using Twiddle
          if (timeToOptimize && steer_pid.IsOptimizationFinished() && frame > (30 / target_speed * 1600) && tuning) {
            steer_pid.startOptimization((30 / target_speed * 1600) * 2, 0.1, {steer_KpD, steer_KiD, steer_KdD});
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
          frame++;

          // Calculating Throttle using PID
          velocity_pid.UpdateError(speed - target_speed);
          throttle = -velocity_pid.TotalError();
          if (throttle < -1) {
            throttle = -1;
          }
          if (throttle > 1) {
            throttle = 1;
          }

          // Print Information
          if (!tuning) {
            std::cout << "Frame," << frame;
            std::cout << ",Current Car Steering Angle," << angle;
            std::cout << ",Current Car Speed," << speed;
            std::cout << ",Current Cross Track Error," << cte;
            std::cout << ",Root Mean Square Error," << rmse.getRMS();
            std::cout << ",Computed Steer Value," << steer_value;
            std::cout << ",Target Speed," << target_speed;
            std::cout << ",Throttle," << throttle;
            std::cout << std::endl;
          }

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