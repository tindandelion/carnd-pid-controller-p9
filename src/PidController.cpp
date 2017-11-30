#include "PidController.hpp"

double PidController::operator()(double measured_value, double delta_t) {
  double error = set_point - measured_value;
  return gains.p * error;
}
