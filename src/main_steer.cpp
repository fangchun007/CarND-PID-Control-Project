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
  if (value > 1)
  {
    //std::cout << std::endl;
    //std::cout << std::endl;
    return 1;
  }
  if (value < -1)
  {
    //std::cout << std::endl;
    //std::cout << std::endl;
    return -1;
  }
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
  
  // Define a huge number as initial error
  double best_err = 1000000.0;
  double err = 0.0;
  std::vector<double> errs = {0.0, 0.0};
  //std::vector<double> p = {0.12, 0.0012, 3.0};
  std::vector<double> p = {0.11, 0.0012, 3.0};
  std::vector<double> dp = {0.01, 0.001, 0.1};
  
  int comp = 0; // index for p, i, d
 
  PID pid_steer;
  // TODO: Initialize the pid variable.
  // pid_steer.Init(0.12, 0.0012, 3.0);
  
  int n_iteration = 0;
  int N = 2500;
  int round = 1;
  
  h.onMessage([&pid_steer, &n_iteration, &N, &comp, &p, &dp, &best_err, &errs, &err, &round]
              (uWS::WebSocket<uWS::SERVER> ws,
               char *data, size_t length,
               uWS::OpCode opCode)
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
                      //************I revised here 16 afternoon ***************//
                      std::string msg_reset = "42[\"reset\"]";
                      
                      if (n_iteration == 0)
                      {
                        std::cout << "Round " << round << ":" << std::endl;
                        p[comp] += dp[comp];
                        std::cout << "  COMPONENT = " << comp << std::endl;
                        std::cout << "  Step 0: " << std::endl;
                        std::cout << "    p[0] = " << p[0] << ", p[1] = " << p[1] << ", p[2] = " << p[2] << std::endl;
                        std::cout << "    dp[0] = " << dp[0] << ", dp[1] = " << dp[1] << ", dp[2] = " << dp[2] << std::endl;
                        pid_steer.Init(p[0], p[1], p[2]);
                        std::cout << "    PID Controller Initialized!" << std::endl;
                      }
                      if (n_iteration == N)
                      {
                        errs[0] = err/N;
                        std::cout << "  Average Error from 1 to N is: " << errs[0] << std::endl;
                        // Reset Simulator to Startpoint
                        //std::string msg = "42[\"reset\"]";
                        ws.send(msg_reset.data(), msg_reset.length(), uWS::OpCode::TEXT);
                        
                        err = 0.0;
                        p[comp] -= 2*dp[comp];
                        std::cout << "  Step " << N << ": " << std::endl;
                        std::cout << "    p[0] = " << p[0] << ", p[1] = " << p[1] << ", p[2] = " << p[2] << std::endl;
                        std::cout << "    dp[0] = " << dp[0] << ", dp[1] = " << dp[1] << ", dp[2] = " << dp[2] << std::endl;
                        pid_steer.Init(p[0], p[1], p[2]);
                        std::cout << "    PID Controller Initialized!" << std::endl;
                      }
                      if (n_iteration == 2*N)
                      {
                        errs[1] = err/N;
                        std::cout << "  Average Error from N to 2N is: " << errs[1] << std::endl;
                        err = 0.0;
                        
                        if (errs[0] < best_err)
                        {
                          best_err = errs[0];
                          p[comp] += 2*dp[comp];
                          //pid_steer.Init(p[0], p[1], p[2]);
                          dp[comp] *= 1.1;
                          std::cout << "Best Error Decreased after Step 1-N." << std::endl;
                          std::cout << "    p[0] = " << p[0] << ", p[1] = " << p[1] << ", p[2] = " << p[2] << std::endl;
                          std::cout << "    dp[0] = " << dp[0] << ", dp[1] = " << dp[1] << ", dp[2] = " << dp[2] << std::endl;
                          comp++;
                          comp = comp % 3;
                          
                          n_iteration = -1;
                          // Output message
                          std::cout << "  Next Process to Component: " << comp << std::endl;
                          std::cout << std::endl;
                        }
                        else
                        {
                          if (errs[1] < best_err)
                          {
                            best_err = errs[1];
                            //pid_steer.Init(p[0], p[1], p[2]); // this step is not necessary
                            dp[comp] *= 1.1;
                            std::cout << "Best Error Decreased after step N-2N." << std::endl;
                            std::cout << "    p[0] = " << p[0] << ", p[1] = " << p[1] << ", p[2] = " << p[2] << std::endl;
                            std::cout << "    dp[0] = " << dp[0] << ", dp[1] = " << dp[1] << ", dp[2] = " << dp[2] << std::endl;
                            comp++;
                            comp = comp % 3;
                            
                            n_iteration = -1;
                            // Output message
                            std::cout << "  Next Process to Component: " << comp << std::endl;
                            std::cout << std::endl;
                          }
                          else
                          {
                            p[comp] += dp[comp];
                            //pid_steer.Init(p[0], p[1], p[2]);
                            std::cout << "No Change for the Best Error! " << std::endl;
                            std::cout << "  The Best Error so far: " << best_err << std::endl;
                            std::cout << "    p[0] = " << p[0] << ", p[1] = " << p[1] << ", p[2] = " << p[2] << std::endl;
                            dp[comp] *= 0.9;
                            std::cout << "    dp values is changed: " << std::endl;
                            std::cout << "    dp[0] = " << dp[0] << ", dp[1] = " << dp[1] << ", dp[2] = " << dp[2] << std::endl;
                            comp++;
                            comp = comp % 3;
                            n_iteration = -1;
                            // Output message
                            std::cout << "  Next Process to Component: " << comp << std::endl;
                            std::cout << std::endl;
                          }
                        }
                        // Reset Simulator to Startpoint
                        
                        ws.send(msg_reset.data(), msg_reset.length(), uWS::OpCode::TEXT);
                        round++;
                      }
                      
                      //************revise end here 16 afternoon ***************//
                      
                      // j[1] is the data JSON object
                      double cte   = std::stod(j[1]["cte"].get<std::string>());
                      double speed = std::stod(j[1]["speed"].get<std::string>());
                      double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                      // Output the present state
                      //std::cout << "Step "<< n_iteration << ": " << std::endl;
                      //std::cout << "  Present: CTE (" << cte << ")    Steering Angle (" << angle << ")" << std::endl;
                      
                      err += cte * cte;
                      
                      // Now we work for a proper steering value using PID controller
                      pid_steer.UpdateError(cte);
                      double steer_value = pid_steer.TotalError();
                      steer_value = truncateAt1(steer_value);
                      
                      
                      // Next write a message and send it to simulator
                      json msgJson;
                      msgJson["steering_angle"] = steer_value;
                      msgJson["throttle"] = 0.4;
                      auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                      
                      // Also output this revised information
                      //std::cout << "  Decision: Adjust Steering Angle (" << steer_value << ")   Adjust Throttle (0.0)" << std::endl;
                      //std::cout << "  Decision: " << msg << std::endl;
                      
                      // Go to the next iteration
                      n_iteration++;
                      
                      /*
                       * TODO: Calcuate steering value here, remember the steering value is
                       * [-1, 1].
                       * NOTE: Feel free to play around with the throttle and speed. Maybe use
                       * another PID controller to control the speed!
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
