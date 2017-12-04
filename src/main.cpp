#include "PID.h"
#include "PidController.hpp"
#include "Simulator.hpp"

class ProductionCarController {
public:
  PidController throttle_controller;
  PidController steer_controller;

  ProductionCarController():
    throttle_controller(PidController::Gains(0.8, 0, 0), 40.0),
    steer_controller(PidController::Gains(0.11, 0.033, 0.11), 0) {}
  
  void operator()(SimulatorResponder& responder, const Measurement& m) {
    double steer_angle = steer_controller(m.cte, m.delta_t);
    double throttle = throttle_controller(m.speed, m.delta_t);
    responder.control(steer_angle, throttle);
  }
};

int main()
{
  const int port = 4567;

  Simulator sim;
  ProductionCarController controller;

  sim.onMeasurement(controller);
  sim.run(port);
}
