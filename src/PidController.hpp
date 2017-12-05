#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H
#include <vector>


class LowPassFilter {
  std::vector<double> buffer;
  int count;

  void append(double value) {
    buffer[count] = value;
    count = (count + 1) % buffer.size();
  }

  double sum() {
    double result = 0;
    for(const double& x: buffer) {
      result += x;
    }
    return result;
  }
  
public:
  LowPassFilter(int size): buffer(size, 0), count(0) { }
  
  double operator()(double new_value) {
    append(new_value);
    return sum() / buffer.size();
  }
};

class PidController {
public:
  struct Gains {
    double p;
    double i;
    double d;

    Gains(double p, double i, double d): p(p), i(i), d(d) {}
  };

  PidController(const Gains& gains, double set_point):
    gains(gains),
    set_point(set_point),
    error_i(0),
    prev_error(0),
    squared_sum_error(0),
    buffer_error_d(LowPassFilter(10)) { }
  
  double operator()(double measured_value, double delta_t);
  double squaredSumError() const { return squared_sum_error; }

private:
  Gains gains;
  double set_point;
  double error_i;
  double prev_error;
  double squared_sum_error;
  LowPassFilter buffer_error_d;
};

#endif
