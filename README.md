# CarND-PID-Control-Project


[//]: # (Image References)

[image1]: ./images_result/pid.png "PID"
[image2]: ./images_result/PID_Breaking.png "PID Breaking"
[image3]: ./images_result/PID_differentCoeff.png "PID differentCoeff"
[image4]: ./images_result/PID_lowErrorAccel.png "PID lowErrorAccel"
[image5]: ./images_result/pidProcess.png "PID Process"

# References

Model Predictive Control (MPC) [Vision-Based High Speed Driving with a Deep Dynamic Observer](https://arxiv.org/abs/1812.02071) by P. Drews, et. al.

Reinforcement Learning-based [Reinforcement Learning and Deep Learning based Lateral Control for Autonomous Driving](https://arxiv.org/abs/1810.12778) by D. Li, et. al.

[ChauffeurNet: Learning to Drive by Imitating the Best and Synthesizing the Worst](https://arxiv.org/abs/1812.03079) by M. Bansal, A. Krizhevsky and A. Ogale


# PID
![alt text][image5]

```Cpp
void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */ 
   
   //Proportional Controller
  p_error = cte;
  
  //Derivative Controller
  if(pid_previous_error == 0)
    	pid_previous_error = cte;
  else
  	d_error = cte - pid_previous_error;
	
  pid_previous_error = cte;
    
  //Integral Controller 
  i_error += cte;
  
  
}
```


```Cpp
double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  pid_result = - Kp* p_error - Kd*d_error - Ki*i_error; 
  return pid_result;  
}
```

# PID Low Error
![alt text][image4]

# PID Control for Speed

![alt text][image2]

```Cpp
double PID::GetRecommendedThrottle(double speed){
    if(fabs(pid_result) < 0.1 || speed < 20)
    {      
      /*if the error is small or the speed is less than 20mph...
      		and the car was breaking: accelerate*/
      if(pid_recommended_throttle < 0)
          pid_recommended_throttle*=-1;
    }
    else if(speed > 25)
    {      
      /*if the error is high, and the speed is more than  25mph...
      		break if the car is still accelerating*/     
      if(pid_recommended_throttle > 0)
          pid_recommended_throttle*=-1;
    }  
  return pid_recommended_throttle;
}
```

# Fast Response PID With Different Coefficients 

![alt text][image3]

```Cpp
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

void PID::UpdateError(double cte)
{  
  if(fabs(cte) > 2.5)
  {
    ChangePIDCoefficients(HIGH_ERROR_PID);    
    pid_recommended_throttle = 4.0;
  }
  else if(fabs(cte) > 1.8)
  {
    ChangePIDCoefficients(MEDIUM_ERROR_PID);    
    pid_recommended_throttle = 3.0;
  }else
  {
    ChangePIDCoefficients(LOW_ERROR_PID);    
    pid_recommended_throttle = 1.0;
  }
  //:
  //:
}
```
