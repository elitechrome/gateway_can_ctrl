#ifndef PID_H
#define PID_H
class  PID
{
public:
  float  Kgain;        // Loop gain parameter
  float  Ti;           // Integrator time constant
  float  Td;           // Differentiator time constant
  float  delT;         // Update time interval

  //saturation
  float lowlim;
  float highlim;

  float  integral;   // Summation of setpoint errors
  float  deriv;      // Previous setpoint error

  PID(float  _Kgain,  float  _Ti, float  _Td, float  _delT, float _lowlim,  float _highlim)
    : Kgain(_Kgain),Ti(_Ti), Td(_Td), delT(_delT), lowlim(_lowlim), highlim(_highlim)
  {
    integral = 0;
    deriv = 0;
  }

float calc(float curr_feedback, float setpt)
{
  double seterr = curr_feedback - setpt;
  // Proportional response
  double pidout = seterr;

  pidout += integral * delT / Ti;

  // drive controller output
  double change = curr_feedback - deriv;
  pidout += change * Td / delT;
  deriv = curr_feedback;

  pidout *= -Kgain;

  // Enforce output limits and anti-windup latch
  if  (pidout > highlim)
    pidout = highlim;
  else if  (pidout < lowlim)
    pidout = lowlim;
  else
    integral += seterr;

  // drive controller output
  return pidout;
}
void reset()
{
  integral = 0;
  deriv = 0;
}

};
#endif // PID_H
