#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "PidController.hpp"
#include <math.h>
#include <time.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

class SimulatorResponder {
  uWS::WebSocket<uWS::SERVER> ws;
  
  void send(const std::string& msg) {
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  }

  void steer(json data) {
    auto msg = "42[\"steer\"," + data.dump() + "]";
    send(msg);
  }
  
public:
  SimulatorResponder(const uWS::WebSocket<uWS::SERVER>& ws): ws(ws) {}

  void control(double steer_angle, double throttle) {
    json msgJson;
    msgJson["steering_angle"] = steer_angle;
    msgJson["throttle"] = throttle;
    steer(msgJson);
  }

  void manual() {
    std::string msg = "42[\"manual\",{}]";
    send(msg);
  }
};

int main()
{
  uWS::Hub h;

  PidController throttle_controller(PidController::Gains(0.8, 0, 0), 40.0);
  PidController steer_controller(PidController::Gains(0.11, 0.033, 0.11), 0);
  time_t timestamp = -1;

  h.onMessage([&throttle_controller, &steer_controller, &timestamp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
      SimulatorResponder responder(ws);
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
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

	  time_t cur_ts = clock();
	  double delta_t = (timestamp < 0) ? 0 : ((float)(cur_ts - timestamp)) / CLOCKS_PER_SEC;
	  timestamp = cur_ts;

	  double steer_angle = steer_controller(cte, delta_t);
	  double throttle = throttle_controller(speed, delta_t);
	  responder.control(steer_angle, throttle);
        }
      } else {
	responder.manual();
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
