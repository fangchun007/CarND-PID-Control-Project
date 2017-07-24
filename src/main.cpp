#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double truncateAt1(double value)
{
  if (value >  1) { return 1; }
  if (value < -1) { return -1; }
  return value;
}
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;
  // Define steer pid controller
  PID pid_steer;
  // Initialize the steering pid variables
  pid_steer.Init(0.21, 8.01034e-05, 3.21709);
  // Define throttle pid controller
  PID pid_throttle;
  // Initialize the throttle pid variables.
  pid_throttle.Init(0.1, 0.002, 0.0);

  h.onMessage([&pid_steer, &pid_throttle]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
              {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                if (length && length > 2 && data[0] == '4' && data[1] == '2')
                {
                  auto s = hasData(std::string(data).substr(0, length));
                  if (s != "") {
                    auto j = json::parse(s);
                    std::string event = j[0].get<std::string>();
                    if (event == "telemetry") {
                      // j[1] is the data JSON object
                      // Receive present vehicle state information
                      double cte   = std::stod(j[1]["cte"].get<std::string>());
                      double speed = std::stod(j[1]["speed"].get<std::string>());
                      //double angle = std::stod(j[1]["steering_angle"].get<std::string>()); // We won't use this info in this version
                      // Use steer pid controller to produce a steering angle based on cte info
                      pid_steer.UpdateError(cte);
                      double steer_value = pid_steer.TotalError();
                      steer_value = truncateAt1(steer_value);
                      // Use throttle pid controller to produce a throttle value
                      const double desired_speed = 45;
                      double speed_error = speed - desired_speed;
                      pid_throttle.UpdateError(speed_error);
                      double throttle_value = pid_throttle.TotalError();
                      if (speed > 50) { throttle_value = 0.32; }
                      // Send the decesion to simulator
                      json msgJson;
                      msgJson["steering_angle"] = steer_value;
                      msgJson["throttle"] = throttle_value;
                      auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                      /*
                       * TODO: Calcuate steering value here, remember the steering value is
                       * [-1, 1].
                       * NOTE: Feel free to play around with the throttle and speed. Maybe use
                       * another PID controller to control the speed!
                       * MY NOTE: The p component of speed controller can be 1/abs(angle - steer_value)
                       */
                    }
                  } else {
                    // Manual driving
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                  }
                }
              });
  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });
  
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
   std::cout << "Connected!!!" << std::endl;
   });
  
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
