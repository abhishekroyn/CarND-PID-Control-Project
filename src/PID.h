#ifndef PID_H
#define PID_H
#include <tuple>
#include <uWS/uWS.h>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_, bool coeff_tune_twiddle);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * Restart the server connection with the simulator.
   * @output None
   */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);

  /**
   * Calculate the PID tunable parameter.
   * @output The PID tunable parameter
   */
  double Twiddle(double twiddle_accumulated_error, double pid_tunable_param);

  /**
   * Get the current value of the counter.
   * @output The counter
   */
  long getCounter();

  /**
   * Set the current value of the counter to zero.
   * @output None
   */
  void resetCounter();

  /**
   * Calculate the average PID error.
   * @output The average PID error
   */
  double AverageError();

  /**
   * Calculate the min PID error.
   * @output The min PID error
   */
  double MinError();

  /**
   * Returns the max PID error.
   * @output The max PID error
   */
  double MaxError();

  /**
   * Returns the PID coefficients.
   * @output The PID coefficients
   */
  std::tuple<double, double, double> Coefficients();

  /**
   * Returns the PID coefficient Kp.
   * @output The PID coefficient Kp
   */
  double getKp();

  /**
   * Assigns value to the PID coefficient Kp.
   * @output None
   */
  void setKp(double current_Kp);

  /**
   * Returns the PID coefficient Ki.
   * @output The PID coefficient Ki
   */
  double getKi();

  /**
   * Assigns value to the PID coefficient Ki.
   * @output None
   */
  void setKi(double current_Ki);

  /**
   * Returns the PID coefficient Kd.
   * @output The PID coefficient Kd
   */
  double getKd();

  /**
   * Assigns value to the PID coefficient Kd.
   * @output None
   */
  void setKd(double current_Kd);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Twiddle
  */
  float tolerance;
  double delta_param;

  /**
   * PID Error counters
   */
  long counter; 
  double error_sum;
  double min_error;
  double max_error;
};

#endif  // PID_H
