#include "PID.h"
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;

	total_cte = 0;

	initialized = false;

	//time steps in twiddle control
	time_steps = 20;
	trailing_cte = 0; // track up to previous 200 time steps of cte
	best_trailing_cte = 0;
	c = 0;
}

void PID::UpdateError(double cte) {
	p_error = -cte*Kp;
	if (!initialized) {
		prev_cte = cte;
		initialized = true;;
	}
	
	double delta_cte = cte - prev_cte;
	prev_cte = cte;
	total_cte += cte;
	d_error = -delta_cte*Kd;
	i_error = -total_cte * Ki;
	std::cout << "delta cte was " << delta_cte << std::endl;

}

void PID::Twiddle(double cte, double& param_to_opp, double& delta_param_to_opp, int& stage) {
	//run for 0 to time_steps - 1
	//the check if parameter change has improved

	if (first_run) {

		c += 1;//increment counter
		best_trailing_cte += cte; // initialize best
		if (time_steps - 1 == c) {
			stage += 1;
			first_run = false;
			c = 0; //reset counter
			//stage 1, increase parameter first pass
			param_to_opp += delta_param_to_opp;
		}
		return;
	}
	
	trailing_cte += cte;




	if (time_steps - 1 == c) {
		if (stage == 1) {
			param_to_opp += delta_param_to_opp;
			stage += 1;
			return;
		}
		else if (stage == 2) {
			//stage 2 check for improvement
			if (trailing_cte < best_trailing_cte) {//improvement
				delta_param_to_opp *= 1.1; // increase momentum in direction of improvment
				stage += 2;// increment stage by 2 to go to next variable
				best_trailing_cte = trailing_cte;
			}
			else {
				param_to_opp -= 2 * delta_param_to_opp;// move in opposite direction
				stage += 1; //increment stage by 1. in stage 3
			}


		}
		else if (stage == 3) {
			if (trailing_cte < best_trailing_cte) {
				best_trailing_cte = trailing_cte;
				delta_param_to_opp *= 1.05;
				stage += 1;
			}
			else {
				param_to_opp -= delta_param_to_opp;
				//there was no improvement after all tests
				//so reset and shrink search interval
				delta_param_to_opp *= 0.95;
				stage += 1;
			}
		}
		c = 0; //reset counter  once enough accumulated for tests
		trailing_cte = 0;//reset trailing cte to accumulate
	}
	c += 1; //increment counter to track toal amount added
	//stage 4 will initiate change in variable outside this function.
}


double PID::TotalError() {
	return p_error + i_error + d_error;
}

