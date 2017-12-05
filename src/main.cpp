#include "PID.h"
#include "PidController.hpp"
#include "Simulator.hpp"

using namespace std;

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

class TwiddleStep {
  PidController::Gains gains;

public:
  TwiddleStep(const PidController::Gains& init_gains): gains(init_gains) {}
  const PidController::Gains& currentGains() const { return gains; }
};

class Twiddler {  
  PidController throttle_controller;
  PidController steer_controller;
  TwiddleStep twiddle_step;
  int max_steps;
  double max_cte;

  void cteOverflow(SimulatorResponder& responder, int step) {
    cout << "CTE overflow!" << endl;
    double overflownError = (steer_controller.squaredSumError() + 1e6) / step;
    nextTwiddleRound(responder, overflownError);
  }

  void nextIteration(SimulatorResponder& responder, int step) {
    double error = steer_controller.squaredSumError() / step;
    nextTwiddleRound(responder, error);
  }

  void normalOperation(SimulatorResponder& responder, const Measurement& m) {
    double steer_angle = steer_controller(m.cte, m.delta_t);
    double throttle = throttle_controller(m.speed, m.delta_t);
    responder.control(steer_angle, throttle);
  }

  void nextTwiddleRound(SimulatorResponder& responder, double error) {
    steer_controller = PidController(twiddle_step.currentGains(), 0);
    responder.reset();
  }
  
public:
  Twiddler(int max_steps, double max_cte):
    throttle_controller(PidController::Gains(0.8, 0, 0), 50),
    steer_controller(PidController::Gains(0, 0, 0), 0),
    twiddle_step(TwiddleStep(PidController::Gains(0.05, 0, 0))),
    max_steps(max_steps),
    max_cte(max_cte) {
    steer_controller = PidController(twiddle_step.currentGains(), 0);
  }
  
  void operator()(SimulatorResponder& responder, const Measurement& m) {
    if (m.step < max_steps) {
      if (m.cte > max_cte) {
	cteOverflow(responder, m.step);
      } else {
	normalOperation(responder, m);
      }
    } else {
      nextIteration(responder, m.step);
    } 
  }
};

int main(int argc, char** argv)
{
  const int port = 4567;
  Simulator simulator;
  ProductionCarController production;
  Twiddler twiddle(1000, 3.0);

  if ((argc > 1) && (string(argv[1]) == "twiddle")) {
    cout << "Running twiddle" << endl;
    simulator.onMeasurement(twiddle);
  } else {
    cout << "Running production" << endl;
    simulator.onMeasurement(production);
  }
  
  simulator.run(port);
}
