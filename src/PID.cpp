#include "PID.h"
#include <math.h>
#include <iostream>
#include <string>


// for convenience
using std::string;

/*
cd CarND-PID-Control-Project/build
mkdir build && cd build
cmake .. && make
./pid
*/

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}




double pid_result = 0;
double pid_previous_error = 0;
double pid_recommended_throttle = 1.0;



void PID::ChangePIDCoefficients(ERROR_PID error)
{
	switch(error){
      case HIGH_ERROR_PID:
  		Kp = 0.1;
  		Ki = 0.0001;
  		Kd = 0.8;
    	break;    
     case MEDIUM_ERROR_PID: 	  
  		Kp = 0.08;
  		Ki = 0.0001;
  		Kd = 0.8;
        break;
     case LOW_ERROR_PID:
  		Kp = 0.05;
  		Ki = 0.00004;
  		Kd = 0.8; 
        break;
    }
}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  /**
   * PID Coefficients
   */ 
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
    /**
   * PID Errors
   */
  p_error = 0;
  i_error = 0;
  d_error = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  std::cout << "CTE: " << cte <<"\t";
  if(fabs(cte) > 2.5)
  {
    ChangePIDCoefficients(HIGH_ERROR_PID);
    std::cout << "HIGH" << "\t";
    pid_recommended_throttle = 4.0;
  }
  else if(fabs(cte) > 1.8)
  {
    ChangePIDCoefficients(MEDIUM_ERROR_PID);
    std::cout << "MEDIUM" << "\t";
    pid_recommended_throttle = 3.0;
  }else
  {
    ChangePIDCoefficients(LOW_ERROR_PID);
    std::cout << "LOW" << "\t";
    pid_recommended_throttle = 1.0;
  }
  
  p_error = cte;
  
  if(pid_previous_error == 0)
    pid_previous_error = cte;
  else
  	d_error = cte - pid_previous_error;
  pid_previous_error = cte;
    
    
  i_error += cte;
  
  
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  pid_result = - Kp* p_error - Kd*d_error - Ki*i_error; 
  return pid_result;  // TODO: Add your total error calc here!
}
double PID::GetRecommendedThrottle(double speed){
	if(fabs(pid_result) < 0.1 || speed < 20)
    {      
      /*if the error is small or the peed less than 20mph...
      		and the car was breaking: accelerate*/
      if(pid_recommended_throttle < 0)
          pid_recommended_throttle*=-1;
      
      std::cout << "ACCEL\t";      
    }else if(speed > 25)
    {      
      /*if the error is high, and the speed more than  25mph...
      		break if the car is still accelerating*/     
      if(pid_recommended_throttle > 0)
          pid_recommended_throttle*=-1;
      
      std::cout << "BREAK\t" ;
    }
  std::cout << "throttle:"<< pid_recommended_throttle << std::endl;
  return pid_recommended_throttle;
}