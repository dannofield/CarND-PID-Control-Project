# CarND-PID-Control-Project


[//]: # (Image References)

[image1]: ./images_result/pid.png "PID"
[image2]: ./images_result/PID_Breaking.png "PID Breaking"
[image3]: ./images_result/PID_differentCoeff.png "PID differentCoeff"
[image4]: ./images_result/PID_lowErrorAccel.png "PID lowErrorAccel"
[image5]: ./images_result/pidProcess.png "PID Process"



# PID
![alt text][image5]

### Tuning method

I basically tuned all the different coefficients by trial and error. 
Because of the dynamics of the system. I found it was a simplier method. 

First I set Ki and Kd values to zero and increase proportional term (Kp) until the car reached to oscillating behavior from one side of the road to the other. 

Once it is oscillating, I adjuste Ki (Integral term) so that oscillations stops, or at least less abrupt. 

Finally, I adjusted D to get fast response.

So the initial coefficients I chose were:
```Cpp
pid.Init(/*Kp_*/0.08, /*Ki_*/0.0001, /*Kd_*/0.8);
```
### PID control loop and Update function 

Every message we get from the behavioral and sensor fusion layer I update the PID control loop to obtain the steering angle and the speed.

```Cpp
pid.UpdateError(cte);
steer_value = pid.TotalError();
throttle = pid.GetRecommendedThrottle(speed);
```

The control loop saves and processes this error

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
After this, it calculates the response based on the error 

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

While the system is experiencing a low error it accelerates constantly with a defined throttle ```double pid_recommended_throttle = 1.0;```

![alt text][image4]

# PID Control for Speed

I implemented the ability for the car to brake when it was going to fast and the error started to increase.

This braking decision is made based on the PID result after calculating the control response (pid_result).

![alt text][image2]

```Cpp
double PID::GetRecommendedThrottle(double speed){
    if(fabs(pid_result) < 0.1 || speed < 20)
    {      
      /*if the error is small or the speed is less than 20mph...
      		and the car was braking: accelerate*/
      if(pid_recommended_throttle < 0)
          pid_recommended_throttle*=-1;
    }
    else if(speed > 25)
    {      
      /*if the error is high, and the speed is more than  25mph...
      		brake if the car is still accelerating*/     
      if(pid_recommended_throttle > 0)
          pid_recommended_throttle*=-1;
    }  
  return pid_recommended_throttle;
}
```

# Fast Response PID With Different Coefficients 

I tried to get creative, particularly around hyperparameter tuning/optimization. 

I noticed that a good, stable algorithm would not be able to respond as fast as you would like when the speed or the error is high.

So I tried to implement 3 different coefficients that the program chooses depending  on the error.

![alt text][image3]

This way, I was able to achieve a greater speed and fast response when the car goes off the center but it behaves very stable when the error is small.

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

# References

Model Predictive Control (MPC) [Vision-Based High Speed Driving with a Deep Dynamic Observer](https://arxiv.org/abs/1812.02071) by P. Drews, et. al.

Reinforcement Learning-based [Reinforcement Learning and Deep Learning based Lateral Control for Autonomous Driving](https://arxiv.org/abs/1810.12778) by D. Li, et. al.

[ChauffeurNet: Learning to Drive by Imitating the Best and Synthesizing the Worst](https://arxiv.org/abs/1812.03079) by M. Bansal, A. Krizhevsky and A. Ogale

