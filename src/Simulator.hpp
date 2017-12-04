#ifndef __SIMULATOR_H
#define __SIMULATOR_H

#include <iostream>
#include <math.h>
#include <time.h>
#include <uWS/uWS.h>
#include "json.hpp"


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

struct Measurement {
  double delta_t;
  double cte;
  double speed;
  double angle;
};

class SimulatorResponder {
  uWS::WebSocket<uWS::SERVER>& ws;
  
  void send(const std::string& msg) {
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  }

  void steer(json data) {
    auto msg = "42[\"steer\"," + data.dump() + "]";
    send(msg);
  }
  
public:
  SimulatorResponder(uWS::WebSocket<uWS::SERVER>& ws): ws(ws) {}

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

class Simulator {
  uWS::Hub hub;
  long timestamp;

  std::string getData(std::string s) {
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
  
public:
  Simulator(): timestamp(-1) {
    hub.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
	std::cout << "Connected!!!" << std::endl;
      });

    hub.onDisconnection([&](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
	uWS::Group<uWS::SERVER>& group = hub;
	group.close();
	std::cout << "Disconnected" << std::endl;
      });
  }

  template <typename EventHandler>
  void onMeasurement(EventHandler& onMeasurement) {
    hub.onMessage([this, &onMeasurement](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
	if (length && length > 2 && data[0] == '4' && data[1] == '2') {
	  SimulatorResponder responder(ws);
	  auto s = getData(std::string(data).substr(0, length));
	  if (s != "") {
	    auto j = json::parse(s);
	    std::string event = j[0].get<std::string>();
	    if (event == "telemetry") {
	      Measurement m;
	      m.cte = std::stod(j[1]["cte"].get<std::string>());
	      m.speed = std::stod(j[1]["speed"].get<std::string>());
	      m.angle = std::stod(j[1]["steering_angle"].get<std::string>());
	      
	      time_t cur_ts = clock();
	      m.delta_t = (timestamp < 0) ? 0 : ((float)(cur_ts - timestamp)) / CLOCKS_PER_SEC;
	      timestamp = cur_ts;

	      onMeasurement(responder, m);
	    }
	  } else {
	    responder.manual();
	  }
	}
      });
  }

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

#endif
