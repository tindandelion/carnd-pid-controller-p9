#ifndef __TWIDDLER_H
#define __TWIDDLER_H

using namespace std;

class TwiddleStep {
  Gains best_result;
  double best_error;
  
  Gains gains;
  Gains increments;

  int current_gain;
  int gain_iteration;
  int _epoch;

  bool isInitialIteration() const { return best_error < 0;}
  bool improvedError(double error) const { return error < best_error; }
  
  void tryNextGain() {
    do  {
      current_gain = (current_gain + 1) % 3;      
    } while (increments[current_gain] == 0);

    gain_iteration = 0;
    if (current_gain == 0) {
      _epoch++;
    }
  }

  void updateGain() {
    if (gain_iteration == 0) {
      gains[current_gain] += increments[current_gain];
      gain_iteration++;
    } else if (gain_iteration == 1) {
      gains[current_gain] -= 2 * increments[current_gain];
      gain_iteration++;
    } else {
      gains[current_gain] += increments[current_gain];
      increments[current_gain] *= 0.9;

      tryNextGain();
      updateGain();
    }
  }
  
public:
  TwiddleStep(const Gains& init, const Gains& increments):
    best_result(init), best_error(-1),
    gains(init), increments(increments),
    current_gain(0), gain_iteration(0) {}
  
  const Gains& current() const { return gains; }
  const Gains& bestResult() const { return best_result; }
  const Gains& incr() const { return increments; }
  
  bool hasFinished() const { return (increments.p + increments.i + increments.d) < 0.01; }
  double bestError() const { return best_error; }
  int epoch() const { return _epoch; }
  
  void next(double error) {
    if (isInitialIteration()) {
      best_error = error;
    } else if (improvedError(error)) {
      best_error = error;
      best_result = gains;
      increments[current_gain] *= 1.1;
      tryNextGain();
    }
    updateGain();
  }
};

class Twiddler {
  PidController throttle_controller;
  PidController steer_controller;
  TwiddleStep twiddle_step;
  int max_steps;
  double max_cte;

  void cteOverflow(SimulatorResponder& responder, int step) {
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

  void reportCurrentResult(double current_error) {
    const Gains& gains = twiddle_step.current();
    const Gains& increment = twiddle_step.incr();

    cout << "Epoch: " << twiddle_step.epoch() << endl;
    cout << "Current error: " << current_error << endl;
    cout << "Current gain values: [" << gains.p << ", " << gains.i << ", " << gains.d << "]" << endl;
    cout << "Increment: [" << increment.p << ", " << increment.i << ", " << increment.d << "]" << endl;

    const Gains& best = twiddle_step.bestResult();
    cout << "Best result before this: [" << best.p << ", " << best.i << ", " << best.d << "]" << endl;
    cout << "Best error before this: " << twiddle_step.bestError() << endl << endl;
  }

  void reportNextRound() {
    const Gains& gains = twiddle_step.current();
    cout << endl << "Next values to try: [" << gains.p << ", " << gains.i << ", " << gains.d << "]" << endl;
  }

  void nextTwiddleRound(SimulatorResponder& responder, double error) {
    if (twiddle_step.hasFinished()) {
      exit(0);
    }

    reportCurrentResult(error);
    
    twiddle_step.next(error);
    reportNextRound();
    
    steer_controller = PidController(twiddle_step.current(), 0);
    responder.reset();
  }
  
public:
  Twiddler(int max_steps, double max_cte, double speed, const Gains& init_gains, const Gains& increment):
    throttle_controller(Gains(0.8, 0, 0), speed),
    twiddle_step(TwiddleStep(init_gains, increment)),
    max_steps(max_steps),
    max_cte(max_cte) {
    steer_controller = PidController(twiddle_step.current(), 0);
  }
  
  void operator()(SimulatorResponder& responder, const Measurement& m) {
    if (m.step < max_steps) {
      if (abs(m.cte) > max_cte) {
	cteOverflow(responder, m.step);
      } else {
	normalOperation(responder, m);
      }
    } else {
      nextIteration(responder, m.step);
    } 
  }
};

#endif
