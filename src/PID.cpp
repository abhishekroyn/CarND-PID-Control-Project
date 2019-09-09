#include "PID.h"
#include <algorithm>
#include <tuple>
#include <uWS/uWS.h>
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

// Initialize PID
void PID::Init(double Kp_, double Ki_, double Kd_, bool coeff_tune_twiddle) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;

  p_error = 0;
  i_error = 0;
  d_error = 0;

  // PID Error counters
  counter = 0;
  error_sum = 0.0;
  min_error = std::numeric_limits<double>::max();
  max_error = std::numeric_limits<double>::min();

  if (coeff_tune_twiddle) {
    tolerance = 0.005;
    delta_param = -0.01;
  }
}

// Update the PID error variables given cross track error
void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  // d_error = cte - previous_cte; previous_cte = cte;

  error_sum += cte;
  counter++;

  if ( cte > max_error ) {
    max_error = cte;
  }
  if ( cte < min_error ) {
    min_error = cte;
  }
}

// Calculate the total PID error
double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  //  return 0.0;  // TODO: Add your total error calc here!
  return  Kp * p_error + Ki * i_error + Kd * d_error;
}

// Restart the server connection with the simulator
void PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

// Calculate the PID tunable parameter
double PID::Twiddle(double twiddle_accumulated_error, double pid_tunable_param) {
  static double current_best_error = 100000;          // random high number to be updated by much lower twiddle_accumulated_error
  static bool is_twiddle_init = false;                // 'false' initially as twiddle expected to be initialized only once
  static bool is_twiddle_reset = true;                // 'true' initially as pid_tunable_param needs to be updated first for further comparison based on accumulated errors
  static double last_pid_tunable_param = 0;
  std::cout << "Current best error is: " << current_best_error << std::endl;
  std::cout << "Twiddle accumulated error is: " << twiddle_accumulated_error << std::endl;
  std::cout << "Delta param is: " << delta_param << std::endl;
  if (!is_twiddle_init) {
  	std::cout << "Twiddle initialized!" << std::endl;
  	current_best_error = twiddle_accumulated_error;
  	is_twiddle_init = true;
  }
  if ((fabs(delta_param) > tolerance)) {
  	if (is_twiddle_reset) {
  		std::cout << "Twiddle reset!-----------------------------" << std::endl;
  		last_pid_tunable_param = pid_tunable_param;
  		pid_tunable_param += delta_param;
      std::cout << "Delta param magnitude REMAINS SAME! PID tunable param magnitude INCREASED!" << std::endl;
  		is_twiddle_reset = false;
      return pid_tunable_param;
  	} else {
  		if (twiddle_accumulated_error < current_best_error) {
  			current_best_error = twiddle_accumulated_error;
  			delta_param *= 1.1;
        std::cout << "Delta param magnitude INCREASED! PID tunable param magnitude to be INCREASED!" << std::endl;
  			is_twiddle_reset = true;
  		} else {
  			if (fabs(last_pid_tunable_param) < fabs(pid_tunable_param)) {
  				last_pid_tunable_param = pid_tunable_param;
  				pid_tunable_param -= 2.0 * delta_param;
  				std::cout << "Delta param magnitude REMAINS SAME! PID tunable param magnitude DECREASED!" << std::endl;
          return pid_tunable_param;
  			} else {
  				last_pid_tunable_param = pid_tunable_param;
  				pid_tunable_param += delta_param;
  				delta_param *= 0.9;
          std::cout << "Delta param magnitude DECREASED! But, PID tunable param magnitude to be INCREASED!" << std::endl;
  				is_twiddle_reset = true;
  			}
  		}
  	}
  }
  return pid_tunable_param;
}

// Get the current value of the counter
long PID::getCounter() {
  return counter;
}

// Set the current value of the counter to zero
void PID::resetCounter() {
  counter = 0;
}

// Calculate the average PID error
double PID::AverageError() {
  return error_sum/counter;
}

// Calculate the min PID error
double PID::MinError() {
  return min_error;
}

// Returns the max PID error
double PID::MaxError() {
  return max_error;
}

// Returns the PID coefficients
std::tuple<double, double, double> PID::Coefficients() {
  return std::make_tuple(Kp, Ki, Kd);
}

// Returns the PID coefficient Kp
double PID::getKp() {
  return this->Kp;
}

// Assigns value to the PID coefficient Kp
void PID::setKp(double current_Kp) {
  this->Kp = current_Kp;
}

// Returns the PID coefficient Ki
double PID::getKi() {
  return this->Ki;
}

// Assigns value to the PID coefficient Ki
void PID::setKi(double current_Ki) {
  this->Ki = current_Ki;
}

// Returns the PID coefficient Kd
double PID::getKd() {
  return this->Kd;
}

// Assigns value to the PID coefficient Kd
void PID::setKd(double current_Kd) {
  this->Kd = current_Kd;
}
