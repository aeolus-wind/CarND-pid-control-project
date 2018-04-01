#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double delta_p;
  double delta_i;
  double delta_d;

  double trailing_cte;
  double best_trailing_cte;
	
  int time_steps;
  int c;

  bool initialized;
  bool first_run;

  double prev_cte;

  double total_cte;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  void Twiddle(double cte,double& param_to_opp, double& delta_param_to_op, int& stage);



  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
