# DiffFlatQuad
This repository is a tody example of employing differential flatness theory to control a simulated 2D planar quadrotor. Given the desired trajectory for the differentially flat outputs of the system, differential flatness theory provides the functions required to compute the full state trajectory and control inputs that lead to that desired output trajectory. Combined with a low-gain PID controller to keep the drone close to the desired path, the differental flatness allows the computation of feed-forward terms that greatly improve the tracking performance.

## The Plant and Its Controllability
The system considered in this project is a planar quadrotor:
<p align="center">
  <img src="docs/plant.png" alt="image" width="45%" height="auto"/>
</p>

and is described using 6 states as follows:

<!-- $
\left[\begin{matrix}\operatorname{\dot{x}_{1}}{\left(t \right)}\\\operatorname{\dot{x}_{2}}{\left(t \right)}\\\operatorname{\dot{x}_{3}}{\left(t \right)}\\\operatorname{\dot{x}_{4}}{\left(t \right)}\\\operatorname{\dot{x}_{5}}{\left(t \right)}\\\operatorname{\dot{x}_{6}}{\left(t \right)}\end{matrix}\right] = 
\left[\begin{matrix}\operatorname{x_{2}}{\left(t \right)}\\- \frac{\operatorname{u_{1}}{\left(t \right)} \sin{\left(\operatorname{x_{5}}{\left(t \right)} \right)}}{m}\\\operatorname{x_{4}}{\left(t \right)}\\- g + \frac{\operatorname{u_{1}}{\left(t \right)} \cos{\left(\operatorname{x_{5}}{\left(t \right)} \right)}}{m}\\\operatorname{x_{6}}{\left(t \right)}\\\frac{\operatorname{u_{1}}{\left(t \right)}}{J}\end{matrix}\right]
$ -->
<p align="center">
  <img src="docs/plant_eq.png" alt="image" width="45%" height="auto"/>
</p>

Here $x_1, x_3, x_5$ are body $x, \ y, \ \theta$ pose parameters, and $x_2, x_4, x_6$ are the corresponding time derivitives ($v_x, v_y, \omega$). The inputs $u_1, \ u_2$ are collective thrust, $F$, and applied torque, $\tau$ by the actuators. These values are linear functions of idividual propeller thrust values:
<!-- $
\begin{align}
\tau =& \frac{F_2-F_1}{l}\\
F =& \frac{F_2+F_1}{2}
\end{align}
$ -->

<p align="center">
  <img src="docs/mixer_eq.png" alt="image" width="20%" height="auto"/>
</p>

where $F_1$ and $F_2$ are the thrust produced by the individual propellers and $l$ is the length from the center of the drone to the motor attachment point. Finally, mass and inertia are represented with $m, J$ and gravity with $g$.

**Note:** Rotational dynamics is decoupled from the position and is described by a double integration of the inertia-normalized body torque. As such, quadrotors usually have a cascade control structure where the inner-loop controller excesises control over the body angular rate and the other-loop control maintains the body pose tracking.  

## Controllability Analysis
Before moving on to the control design, we first need to investigate the controllability of the plant under study. First, note that the plant may be put into control-affine forms as:

<!-- \mathbf{\dot{x}} = &\mathbf{f}(\mathbf{x})+\mathbf{g_1}(\mathbf{x})u_1+ \mathbf{g_2}(\mathbf{x})u_2\\
\mathbf{f} = \left[\begin{matrix}\operatorname{x_{2}}\\0\\\operatorname{x_{4}}\\- g\\\operatorname{x_{6}}\\0\end{matrix}\right], \
&\mathbf{g}_1 = \left[\begin{matrix}0\\- \frac{\sin{\left(\operatorname{x_{5}}{\left(t \right)} \right)}}{m}\\0\\\frac{\cos{\left(\operatorname{x_{5}}{\left(t \right)} \right)}}{m}\\0\\0\end{matrix}\right], \
\mathbf{g}_2 = \left[\begin{matrix}0\\0\\0\\0\\0\\\frac{1}{J}\end{matrix}\right] -->
<p align="center">
  <img src="docs/plant_decom_eq.png" alt="image" width="60%" height="auto"/>
</p>

