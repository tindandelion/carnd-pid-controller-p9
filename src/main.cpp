#include "PidController.hpp"
#include "Simulator.hpp"
#include "Twiddler.hpp"


class ProductionCarController {
public:
  PidController throttle_controller;
  PidController steer_controller;

  ProductionCarController(const Gains& steer_gains, double speed):
    throttle_controller(Gains(0.8, 0, 0), speed),
    steer_controller(steer_gains, 0) {}
  
  void operator()(SimulatorResponder& responder, const Measurement& m) {
    double steer_angle = steer_controller(m.cte, m.delta_t);
    double throttle = throttle_controller(m.speed, m.delta_t);
    responder.control(steer_angle, throttle);
  }
};


int main(int argc, char** argv)
{
  const int port = 4567;
  Simulator simulator;
  ProductionCarController production(Gains(0.31, 1.1, 0.01), 30.0);
  Twiddler twiddle(3500, 3.0, 40.0, Gains(0.2, 1.0, 0.01), Gains(0.1, 0.1, 0.1));

  if ((argc > 1) && (string(argv[1]) == "twiddle")) {
    cout << "Running twiddle" << endl;
    simulator.onMeasurement(twiddle);
  } else {
    cout << "Running production" << endl;
    simulator.onMeasurement(production);
  }
  
  simulator.run(port);
}
