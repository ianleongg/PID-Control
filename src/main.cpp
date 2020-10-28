#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "twiddle.h"
#include "twiddle.cpp"

// for convenience
using nlohmann::json;
using std::string;

//twiddle counter for steering
static int count_s = 0;

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

int main() {
  uWS::Hub h;

  PID pid_s;
  Twiddle twiddle_s;

  //after twiddle, initialize new values
  std::vector<double> values;
  values = {0.143143, 0.0028836, 3.1};
 
  //initiliaze pid and twiddle with values
  pid_s.Init(values[0], values [1], values[2]);
  twiddle_s.Init(values[0], values [1], values[2]);

  h.onMessage([&pid_s, &twiddle_s](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, throttle_value;

          //STEERING
          //update cte at each step and get steer value from total error
          pid_s.UpdateError(cte);
          steer_value = pid_s.TotalError();
          //clip it to [-1,1]
          steer_value = fmax(-1, steer_value);
          steer_value = fmin(1, steer_value);

          //THROTTLE
          //slow down when cte is high
          if (fabs(cte) > 1.0) {
            throttle_value = 0.3;
          } 
          //else, throttle = inverse of steering, max 100mph
          else {
            throttle_value = fmin(1.0 / fabs(steer_value), 100.0);
            //normalize throttle value to [0.45, 1.0], max 100mph
            throttle_value = ((1.0 - 0.45) * throttle_value) / (100.0 + 0.45);
          }
         
        //TWIDDLE 
        // let's use a sample size of 100 for twiddle
        bool sample_state_s = (++count_s % 100 == 0);

        // twiddle algorithm with tolerance of 0.2
        double tolerance = 0.2;
        
        //twiddle steering
        bool tolerance_state_s = false;
        if (!tolerance_state_s) {
          twiddle_s.IncrementCount(cte);
          if (sample_state_s) {
            std::vector<double> p_params = twiddle_s.UpdateParams();
            if (twiddle_s.GetTolerance() < tolerance) {
              tolerance_state_s = true;
            }
            else {
              pid_s.Init(p_params[0], p_params[1], p_params[2]);
            }
          }
        }

          // DEBUG
         //std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
          //          << "Throttle Value: " << throttle_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
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