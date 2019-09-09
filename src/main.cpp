#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <tuple>

// for convenience
using nlohmann::json;
using std::string;

// global variable for choosing PID coefficients tuning by Twiddle method
bool coeff_tune_twiddle = false;

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

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */

  double init_Kp, init_Ki, init_Kd;

  // multiple ways of execution,
  // 1. using manually tuned default hyperparameters from code [./pid]
  // 2. using manually input hyperparameters [./pid -0.09 -0.0001 -1.80]
  // 3. using manually input hyperparameters to be further tuned by Twiddle [./pid -0.09 -0.0001 -1.80 twiddle]
  // 4. using manually input hyperparameters after further tuning by Twiddle [./pid -0.2504 -0.0001 -1.822]
  if (argc > 1) {
    init_Kp = atof(argv[1]);
    init_Ki = atof(argv[2]);
    init_Kd = atof(argv[3]);
    if (argc > 4) {
      std::string coeff_tune_method = argv[4];
      if (coeff_tune_method.compare("twiddle") == 0) {
        coeff_tune_twiddle = true;
      }
    }
  } else {
    init_Kp = -0.09;            // manually tuned default proportional hyperparameter
    init_Ki = -0.0001;          // manually tuned default integral hyperparameter
    init_Kd = -1.80;            // manually tuned default derivative hyperparameter
  }

  // Initialize PID coefficients
  pid.Init(init_Kp, init_Ki, init_Kd, coeff_tune_twiddle);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
//          double speed = std::stod(j[1]["speed"].get<string>());
//          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          // Update the PID error variables given cross track error
          pid.UpdateError(cte);

          // Calculate the total PID error
          steer_value = pid.TotalError();

          // static variables for twiddle
          static double pid_tunable_param = 0.0;
          static double current_param = 0.0;
          static double twiddle_accumulated_error = 0.0;

          // PID coefficients tuning by Twiddle method
          if (coeff_tune_twiddle) {
            if (pid.getCounter() > 2500) {          // 2500 counts refer to 2500 message events received from the simulator, which is also equivalent to approximately 1 complete lap of the track in the simulator at the default set throttle value of 0.3
              std::cout << "============= Using Twiddle to update! =============" << std::endl;
              pid_tunable_param = pid.getKp();      // Tuning hyperparameters one at time
              current_param = pid.Twiddle(twiddle_accumulated_error, pid_tunable_param);
              if (current_param != pid_tunable_param) {
                pid.setKp(current_param);
                pid.Restart(ws);
                pid.resetCounter();
                twiddle_accumulated_error = 0.0;
              }
            } else {
              twiddle_accumulated_error  += pow(cte, 2);      // to ensure accumulation of both postive and negative cross track error in absolute sense
            }
          }

          // DEBUG
//          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
//                    << std::endl;
          double final_Kp, final_Ki, final_Kd;
          std::tie(final_Kp, final_Ki, final_Kd) = pid.Coefficients();
//          std::cout << "Counter : " << pid.getCounter() 
//                    << "     Average Error : " <<  pid.AverageError() 
//                    << " [" << pid.MinError() << ", " << pid.MaxError() << "]" 
//                    << "     PID Coefficients : " 
//                    << " (" << final_Kp << ", " << final_Ki << ", " << final_Kd << ")" 
//                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
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
