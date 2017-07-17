#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID()
{
  p_error_ = i_error_ = d_error_ = 0.0;
  Kp_ = Ki_ = Kd_ = 0.0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void PID::UpdateError(double cte)
{
  d_error_ = cte - p_error_;
  p_error_ = cte;
  i_error_ += cte;
}

double PID::TotalError() {
  return - p_error_ * Kp_ - d_error_ * Kd_ - i_error_ * Ki_;
}

