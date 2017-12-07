#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

struct Gains {
  double p;
  double i;
  double d;
  
  constexpr Gains(double p, double i, double d): p(p), i(i), d(d) {}
  
  double& operator[](int index) {
    switch(index) {
    case 0: return p;
    case 1: return i;
    case 2: return d;
    default: throw "Invalid index";
    }
  }
};

class PidController {
  Gains gains;
  double set_point;
  double error_i;
  double prev_error;
  double squared_sum_error;

public:
  PidController(): PidController(Gains(0, 0, 0), 0) {}
  PidController(const Gains& gains, double set_point):
    gains(gains),
    set_point(set_point),
    error_i(0),
    prev_error(0),
    squared_sum_error(0) { }
  
  double operator()(double measured_value, double delta_t) {
    double error = set_point - measured_value;
    double error_d = delta_t != 0 ? (error - prev_error) / delta_t : 0;

    error_i += error * delta_t;
    prev_error = error;
    squared_sum_error += error*error;
  
    return gains.p * error + gains.i * error_i + gains.d * error_d;
  }
  
  double squaredSumError() const { return squared_sum_error; }
};

#endif
