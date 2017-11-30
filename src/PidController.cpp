#include "PidController.hpp"

double PidController::operator()(double measured_value, double delta_t) {
  double error = set_point - measured_value;
  double error_d = delta_t != 0 ? (error - prev_error) / delta_t : 0;

  error_i += error * delta_t;
  error_d = buffer_error_d(error_d);
  prev_error = error;
  
  return gains.p * error + gains.i * error_i + gains.d * error_d;
}
