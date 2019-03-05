/*
 * PidController.cpp
 *
 *  Created on: 23 Feb 2017
 *      Author: sleutene
 */

#include <arp/PidController.hpp>
#include <stdexcept>
#include <cmath>

namespace arp {
using namespace std;
// Set the controller parameters
void PidController::setParameters(const Parameters & parameters)
{
  parameters_ = parameters;
}

// Implements the controller as u(t)=c(e(t))
double PidController::control(uint64_t timestampMicroseconds, double e,
                              double e_dot)
{
  uint64_t time_diff;
  time_diff = (timestampMicroseconds - lastTimestampMicroseconds_)* (pow(10,-6));

  // Clip the time step to a maximum of 0.1s to avoid huge integrated error
  if(time_diff > 0.1) time_diff = 0.1;

  double output;
  output = parameters_.k_p * e + parameters_.k_i * integratedError_
            + parameters_.k_d * e_dot;
  //std::cout << "min output"<< output << minOutput_<< std::endl;
  if (output < minOutput_) {
    output = minOutput_; // Clamp and DO NOT Integrate error
  }
  else if (output > maxOutput_) {
    output = maxOutput_; // Clamp and DO NOT Integrate error
  }
  else {
    integratedError_ += e * time_diff;
  }

  lastTimestampMicroseconds_ = timestampMicroseconds;
  return output;
}

// Reset the integrator to zero again.
void PidController::setOutputLimits(double minOutput, double maxOutput)
{
  minOutput_ = minOutput;
  maxOutput_ = maxOutput;
}

/// \brief
void PidController::resetIntegrator()
{
  integratedError_ = 0.0;
}

}  // namespace arp
