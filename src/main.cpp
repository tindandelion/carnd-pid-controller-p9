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

  void reset() {
    std::string msg = "42[\"reset\",{}]";
    send(msg);
  }
};

struct Measurement {
  double delta_t;
  double cte;
  double speed;
  double angle;
};

class Simulator {
public:
  uWS::Hub& hub;

  Simulator(uWS::Hub& hub): hub(hub) {}

  void run(int port) {
    bool listening = hub.listen(port);
    if (listening) {
      std::cout << "Listening on port " << port << std::endl;
    } else {
      std::cerr << "Unable to listen on port " << port << std::endl;
      return;
    }
    hub.run();
  }
};

int main()
{
  uWS::Hub h;
  Simulator sim(h);
  
  PidController throttle_controller(PidController::Gains(0.8, 0, 0), 40.0);
  PidController steer_controller(PidController::Gains(0.11, 0.033, 0.11), 0);
  time_t timestamp = -1;
  long step = 0;

  auto f = [&throttle_controller, &steer_controller, &step](SimulatorResponder& responder, const Measurement& m) {
    double steer_angle = steer_controller(m.cte, m.delta_t);
    double throttle = throttle_controller(m.speed, m.delta_t);
    if (++step > 1000) {
      responder.reset();
      step = 0;
    } else {
      responder.control(steer_angle, throttle);      
    }
  };

  h.onMessage([&timestamp, &f](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
	  Measurement m;
          m.cte = std::stod(j[1]["cte"].get<std::string>());
          m.speed = std::stod(j[1]["speed"].get<std::string>());
          m.angle = std::stod(j[1]["steering_angle"].get<std::string>());

	  time_t cur_ts = clock();
	  m.delta_t = (timestamp < 0) ? 0 : ((float)(cur_ts - timestamp)) / CLOCKS_PER_SEC;
	  timestamp = cur_ts;

	  f(responder, m);
        }
      } else {
	responder.manual();
      }
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
      uWS::Group<uWS::SERVER>& group = h;
      group.close();
      std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  sim.run(port);
}
