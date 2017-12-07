# PID controller

This project is a part of Udacity's *Self-Driving Car Nanodegree* program. The
goal of the project is to implement a PID (Proportional-Integral-Drivative)
controller that keeps the car on the track in the simulator. 

# Reflection

A PID controller is a feedback loop controller continuously calculates an error
as a difference between a desired setpoint and a measured process variable, and
applies a correction based on proportional, integral, and devrivative terms. In
this project, the object to control is a car in the simulator, the process
variable is the offset of the car from the center of the road, and the control
is an angle of the steering wheel that's applied to keep the car centered. 

## P component

**P** is the *proportional* term in the equation. The value of the P coefficient
  reflects how strongly the car will react to the deviations from the desired
  trajectory. In general, the bigger the P value, the bigger the correction
  signal will be. The downside of having big P values is that the whole process
  becomes unstable because the car increasingly overshoots the desired position.
  
**I** is the *integral* term in the equation. It is used to account for a
  residual systematic error that may exist due to the specifics of the
  controlled device (e.g. the imsalignments of the wheels), or the enviroment
  (e.g. strong side wind). The effect of the integral term is that it
  accumulates over time, thus providing the control signal that eventually
  brings the error value to zero. 
  
**D** is the *derivative* component that's designed to counter the overshoot
  effect and stabilize the device. Essentially, it's the the term that's
  proportional to the speed of change in the measurements in response to the
  control. It provides the damping effect that prevents the car from constant
  overshooting. 
  
# Tuning the parameters

In general, finding the optimal P, I, and D parameter set is a challenging
trial-and-error task. In this project, I used the "Twiddle" algorithm to adjust
the parameters. The essense of the Twiddle (which essentially is a variation of
a coordinate accent) is that it adjusts the coefficients, trying to find the
lowest overall error. This algorithm is useful when fine-tuning the roughly
estimated parameters, although somewhat time-consuming and prone to getting stuck in
the local optimum.

# Implementation details

There are 3 principal components in my implementation: 

### `Simulator` 

This is a class that encapsulates the communication between the controller and
the simulator. It captures the data packages from the simulator, unpacks them
from JSON into an internal data structure, and passes along to the
`onMeasurement` event handler. The handler is supposed to process the
measurement and apply the control through the instance of a helper
`SimulatorResponder`, that's also passed along. The control values then get
packed into JSON format and sent back to the simulator. Additionally, the event
handler can command the simulator to reset, which is used in the optimization
mode.

### `ProductionCarController`

This is a 'production' controller that runs on a predefined set of P, I, and D
parameters. Inside, it encapsulates 2 separate PID controllers (instances of
`PidController` class), one for steering value, and one for speed. As the
primary focus of this project is on the steering, the speed controller is a
simple P-controller that just keeps the speed constant at 30 mph.

### Twiddler 

This is an implementation of the fine-tuining 'Twiddle' algorithm. Given initial
values for P, I, and D gains, it runs the simulation for a while, records the
overall error, and then restarts the simulation with the adjusted set of
values. The helper `TwiddleStep` class keeps track of which parameter is being
tried in every round. The process stops when the change in values becomes small
enough. 


To run the program in the production mode, run `./pid` without any
parameters. For optimization, run `./pid twiddle`.









