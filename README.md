# CarND-Model-Predictive-Control
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This is the project repository for **Project No. 5 Model Predictive Control**, part of Term 2 _Sensor Fusion_ of Udacity Self-Driving Car Nanodegree program, submitted by Albert Killer in June 2017. 


## MPC for Autonomous Driving

Model predictive control (MPC) is an advanced method of process control for complex dynamic systems, with self-driving cars beeing one them. By applying a time-discrete dynamic model it allows us to predict future behaviour of a system in accordance to its input signals, as vehicle steering or throttle. 

While an optimal output signal is calculated for *N* steps in advance, the optimization calculation itself is repeated over again after each *N*-th processing step on basis of the currently measured states, i.e. the position of the vehicle on the road. In order to effectively optimize predictions a cost function is set up to minimize erros. These errors, consisting of deviations to references values, are referred to as *cross track error* and *steering angle error*. 


## Model

To build the MPC for this project a **kinematic model** is applied, which takes into account following variables describing the car’s state and actuator inputs: 

**States**: [ *x*, *y*, *ψ*, *v* ] ... *x* and *y* beeing the location coordinates, *psi* the vehicle's orientation angle and *v* the velocity. 

**Actuators**: [ *δ*, *a* ] ... *delta* beeing the steering angle, an *a* the vehicle's acceleration

To set up **model constraints** based on the vehicle model following equations are used: 

![Model constraints](eq-1.png?raw=true "Model constraints")

*Lf* refers to physical characteristics of the vehicle, changing i.e. with size depending of the car’s center of gravity. 

To generate actuator outputs the MPC predicts the vehicle’s path and then tries to minimize the differences to the reference trajectory. As mentioned above the errors of interest are the cross track error *cte* and the orientation error *eψ*: 

![Errors](eq-2.png?raw=true "Errors")

The **cost function** is designed to set different emphasis on the effect of certain states actuations. This  way a focus can be set on the car keeping on track over the regulation to a constant speed level for example. In addition erratic behaviour like sudden and extreme changes of steering angle can be avoided systematically.

The presented algorithm uses the **optimization solver** *[Ipopt](https://projects.coin-or.org/Ipopt)*. The solver takes all state and actuator variables in a single vector and returns control inputs that minimizes the cost function. 


## Timestep Length and Elapsed Duration 

How far into the future our prediction should be made, is defined by the product of the timestep length *N* and the elapsed duration *dt* and refered to as the prediction horizon *T*. It should be as large as possible while *dt* should be as small as possible.  At the same time the smaller the duration *dt* becomes, the heavier to compute, because all the MPC's calculations have to be done within one duration *dt*. *N* also sets the number of variables optimized by the MPC, increasing computational costs as well:

```c++
size_t n_vars = N * 6 + (N - 1) * 2;
```

It has been experimentally verified that the best results for this simulation are achieved by:

```c++
size_t N = 9; // okay between 7 and 10 
double dt = 0.1;
```

Higher values of *N* slower down the controller's reaction time, which finally leads to the car getting off the road in critical situations (as approaching tight turns at higher speed). Lower values of *N* limit the number of predicted states and therefor taking away the ability to adjust steering appropriately while approaching situations which demand certain foresight, like tight turns for instance. 

Higher values of *dt* on the other hand first result in slowing down the speed in curves drastically making the controller ineffective. And finally lower values of *dt* cause oscillation which leads to the car getting off the track very quickly.



## Polynomial Fitting and MPC Preprocessing

When getting input from the simulator, the presented algorithm reads in the vehicle’s states and actuators. In order to simplify further calculations a **transformation into a different orientation space** is applied by shifting the car’s reference angle to 90 degrees. Afterwards a **polynomial** describing the car’s trajectory can be **fit to waypoints** ( *x*, *y* ) using vectors of the *[Eigen library](http://eigen.tuxfamily.org/index.php?title=Main_Page)* for linear algebra. By evaluating the polynomial’s coefficients, in this case at the initial point due to above mentioned transformation, the cross track error is calculated. As shown before the orientation error is described by following equation: *eψ* = *ψ* − *ψdes*. The latter *ψdes* is acquired by applying trignometry, leading to the, as well simplified, orientation error of: 

```c++
double epsi = -atan(coeffs[1]);
```


In order to allow **visual debugging** the simulator is fed with waypoints for a reference line (yellow) and the trajectory currently predicted by the MPC (green).



![Screenshot of simulation result](Screenshot%20from%202017-06-19%2021-37-11.png?raw=true "Screenshot of simulation result")


## Handling Latency

In order to get this simulation a little closer to a realistic scenario, we have to consider latency. Latency describes the delay caused by the time needed for a controller's actuation output signal to result in an actual steering maneuver of the vehicle.

One approach to take this effect into account is to predict the future state after the delay caused by the latency using the **kinematic equations** mentioned above. This prediction should then be the input state for the MPC. 

As following videos, taken from the simulator, show, ignoring latency slows down steering reaction and amplifies oscillation.  

* Simulation, *considering latency* of 100 ms: https://youtu.be/9pfyWmesjhY

* Simulation, *ignoring latency* of 100 ms: https://youtu.be/UBLjkUbnDw4


## Conclusion

Advantages of MPC:
* Predictive ability on basis of calculating future events (advantage over controllers like PID i.e.)
* Considers disturbances well
* Tuning different parameters *possible*
* Able to accurately deal with latency on basis of a vehicle model

Disadvantages of MPC:
* Computantionally expensive
* Tuning different parameters *required*


Want to know more about MPC? Have a look at this one: https://en.wikipedia.org/wiki/Model_predictive_control
