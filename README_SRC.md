In this file, I explain how the PID controllers are designed in this project and how the final hyperparameter are chose. 

## 1. How the PID controllers are designed in my implementation?

Two PID controllers are implemented in this project. One is for the steer control. The other one is for throttle control. The idea for designing the throttle controller comes from the implementation drive.py of the Bahavioral Cloning Project. One can refer to the following link.

https://github.com/udacity/CarND-Behavioral-Cloning-P3/blob/master/drive.py

Here, we explain the effect of P, I, D, components in the PID controller for steering as an example. 

**PROPORTIONAL COMPONENT**. Since the purpose of a steering controller is to pull the vehicle back to and maintain it around the reference line. The proportional component is set to cross track error (cte) of the vehicle. The farther the vehicle away from the reference line, the larger steering angle is suggested by the controller. In our case, a small value close zero didn’t turn the vehicle enough and possibly the car can go off the road. A large value, e.g., 1, would make the car oscillate too much. As I see from test running on simulator, if the car cannot make a sharp turn and runs over the lane line, then slightly increase this P value would help.

**DERIVATIVE COMPONENT**. In math, derivatives are used to describe the change rate of curves. In our case, when p value is 0.11, and the other two are zero, the car starts with oscillations and eventually goes off the road. To make the car approach smoothly to the reference line and more stable, a derivative component is needed. The sharper of the curve the vehicle made, more errors will be advised by the controller to prevent a steep curve and make the car drives smoother. A good combination of P value and D value, in our case, can result in a  

**INTEGRAL COMPONENT**. The I term is designed to correct for the wheel drift. As we found, the car drives well when the I value is small. This suggest a small wheel drift for our simulator.


## 2. Hyperparameter Tuning / Optimization.

The P, I, D coefficients are chosen through a combination of manual tuning (in the setting of initial values and adjusting of the length of experimental section) and a revised twiddle algorithm. Here, we only explain the implementation of twiddle algorithm main_steer.cpp. 

In case you wish to try it by yourself, please just change the original main.cpp to some other names, change its name to main.cpp and run it. 

Once you’ve ever tried parameter manual tuning and run your algorithm on simulator after each revision, you will find it is a time consuming and boring task. Our aim is to finish an algorithm so that the computer itself can figure out good parameters for us. 

To this purpose, the first thing we need to take care is that uWs::Hub.run() never stop as one wished, once it was operated. This will cause difficulty if we want to implement a twiddle algorithm directly because of the nest structural for-loop in twiddle. In our implementation, we expand the for-loop to avoid this problem. In return, we lose some time on unnecessary calculations. However, we still get compensation, since we can now always choose a better direction on which the cost function has the biggest decrease. 

The second difficulty is how to set the initial values. Random search, as pointed out by my tutor Alan, is a good candidate. It works well with our twiddle implementation. Actually, when we start with the initial parameters p = (Kp, Ki, Kd) = (0, 0, 0), dp = (dpp, dpi, dpd) = (1, 1, 1) and set N = 2000, the algorithm can obtain a decent result after around 26 rounds. We get p = (0.106148, 0, 4.39876) and dp = (0.864536, 0.430467, 1.29147). 

To accelerate the computation, we suggest the following method. Let N denote the frames. Then the larger the number N is, the further the car can drive. We will start with a short distance and a small throttle, such as N = 1000 and set desired speed to 40. Then try it with the algorithm. Once obtained a good parameter, we reset N to 2000, and use the obtained parameter as the new initial value, and run the algorithm. Next, we set desired speed to 50 and tune the throttle parameters. In this way, one can procced similarly to N = 2000, 3000, and higher desired speed etc., and finally one can obtain the final parameters. It is worth to point out in order to obtain a higher speed, we need to redesign the throttle PID controller. 
