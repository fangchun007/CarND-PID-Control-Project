In this file, I explain how the PID controllers were designed in this project and how the final hyperparameter were chosen. 

## 1. How the PID controllers were designed in my implementation?

Two PID controllers were implemented in this project. One is for the steer control. The other one is for the throttle control.
The idea for designing the throttle controller comes from implementation drive.py of the Bahavioral Cloning Project. One can 
refer to the following link.

https://github.com/udacity/CarND-Behavioral-Cloning-P3/blob/master/drive.py

Here, we explain the effect of P, I, D, components in the PID controller for steering as an example. 

PROPORTIONAL COMPONENT. Since the purpose of a steering controller is to pull the vehicle back to and maintain it around the 
reference line. The proportional component is set to cross track error (cte) of the vehicle. The farther the vehicle away from
the reference line, the larger steering angle is suggested by the controller. In our case, a small value close zero didn’t 
turn the vehicle enough and possibly the car can go off the road. A large value, e.g., 1, would make the car oscillate too
much. As I see from test running on simulator, if the car cannot make a sharp turn and runs over the lane line, then slightly
increase this P value would help.

DERIVATIVE COMPONENT. In math, derivatives are used to describe the change rate of curves. In our case, when p value is 0.11, 
and the other two are zero, the car starts with oscillations and eventually goes off the road. To make the car approach smoothly
to the reference line and more stable, a derivative component is needed. The sharper of the curve the vehicle made, more errors
will be advised by the controller to prevent a steep curve and make the car drives smoother. A good combination of P value and
D value, in our case, can result in a  

INTEGRAL COMPONENT. The I term is designed to correct for the wheel drift. As we found, the car drives well when the I value is
small. This suggest a small wheel drift for our simulator.


## 2. Hyperparameter Tuning / Optimization.

