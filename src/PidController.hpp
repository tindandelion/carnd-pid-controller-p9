#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

class PidController {
public:
  struct Gains {
    double p;
    double i;
    double d;

    Gains(double p, double i, double d): p(p), i(i), d(d) {}
  };

  PidController(const Gains& gains): gains(gains) {}
  double operator()(double measured_value);

private:
  Gains gains;
};

#endif
