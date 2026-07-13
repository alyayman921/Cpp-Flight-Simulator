# LOCKHEED MARTIN'S C5A Autopilot System 

## Discretization warning
> this model does not discretize the model same way matlab does, so if you find yourself wondering where the difference came from, it's probably this

**Small Timesteps C++ vs Matlab**
![Cpp vs Matlab at 0.001](Documentation/V0.1/CppVsMatlab150@ts0.001.png)
**Bigger Timesteps C++ vs Matlab**
![Cpp vs Matlab at 0.001](Documentation/V0.1/CppVsMatlab150@ts0.01.png)

## AI Usage
Ai was used in this project to do the logging hand the handling of files, it was also used in developing the rk4 solver and linking it to the Rigid Body Dynamics equations
this is a learning project, i'm trying to enhance my c++ skills so there's no point of prompting the ai to make it faster


## Build 
```bash
g++ Autopilot.cpp RBDEqns.cpp -o Autopilot -I/usr/include/eigen3 -lm -lxlsxio_read
```
## Controller Implementation Choice
- This is a pseudo-Continuous Time project, if matlab can do it i can do it too with worse results!
so the controller is implemented like so, if a state is to be derived (multiplied by s), the implementation is like so, this time step we set the state_derivative = sum of non derivated states and next time step you get the state by doing state += state_derivative * dt
- Most Likely the Servos will not be implemented for now, too much work for a 1st order delay currently
