# DiffFlatQuad
This repository is a tody example of employing differential flatness theory to control a simulated 2D planar quadrotor. Given the desired trajectory for the differentially flat outputs of the system, differential flatness theory provides the functions required to compute the full state trajectory and control inputs that lead to that desired output trajectory. Combined with a low-gain PID controller to keep the drone close to the desired path, the differental flatness allows the computation of feed-forward terms that greatly improve the tracking performance.

## The Plant
The system considered in this project is a planar quadrotor:
<p align="center">
  <img src="docs/plant.png" alt="image" width="45%" height="auto"/>
</p>

This system is described using 6 states as follows:

<!-- $
\left[\begin{matrix}\operatorname{\dot{x}_{1}}{\left(t \right)}\\\operatorname{\dot{x}_{2}}{\left(t \right)}\\\operatorname{\dot{x}_{3}}{\left(t \right)}\\\operatorname{\dot{x}_{4}}{\left(t \right)}\\\operatorname{\dot{x}_{5}}{\left(t \right)}\\\operatorname{\dot{x}_{6}}{\left(t \right)}\end{matrix}\right] = 
\left[\begin{matrix}\operatorname{x_{2}}{\left(t \right)}\\- \frac{\operatorname{u_{1}}{\left(t \right)} \sin{\left(\operatorname{x_{5}}{\left(t \right)} \right)}}{m}\\\operatorname{x_{4}}{\left(t \right)}\\- g + \frac{\operatorname{u_{1}}{\left(t \right)} \cos{\left(\operatorname{x_{5}}{\left(t \right)} \right)}}{m}\\\operatorname{x_{6}}{\left(t \right)}\\\frac{\operatorname{u_{1}}{\left(t \right)}}{J}\end{matrix}\right]
$ -->
<p align="center">
  <img src="docs/plant_eq.png" alt="image" width="45%" height="auto"/>
</p>

where $x_1, x_3, x_5$ are body $x, \ y, \ \theta$ body pose and $x_2, x_4, x_6$ are the corresponding time derivitives ($v_x, v_y, \omega$). The inputs $u_1, \ u_2$ are collective thrust, $F$, and applied torque, $\tau$ by the actuators. These values are linear functions of idividual propeller thrust values as follows: 

