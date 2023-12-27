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

Here, we provide a simple simulator based on Pygame that can be instantiated as follows: 

```python
from DiffFlatQuad.robot import PlanerQuadrotor
quadrotor = PlanerQuadrotor(rendering=True, dt=1e-2)
#The simulation loop
while quadrotor.running():
    quadrotor.step(T=0.0000, F=9.8)
    time.sleep(0.01)
```

### Controllability Analysis
Before moving on to the control design, we first need to investigate the controllability of the plant under study. First, note that the plant may be put into control-affine forms as:

<!-- \mathbf{\dot{x}} = &\mathbf{f}(\mathbf{x})+\mathbf{g_1}(\mathbf{x})u_1+ \mathbf{g_2}(\mathbf{x})u_2\\
\mathbf{f} = \left[\begin{matrix}\operatorname{x_{2}}\\0\\\operatorname{x_{4}}\\- g\\\operatorname{x_{6}}\\0\end{matrix}\right], \
&\mathbf{g}_1 = \left[\begin{matrix}0\\- \frac{\sin{\left(\operatorname{x_{5}}{\left(t \right)} \right)}}{m}\\0\\\frac{\cos{\left(\operatorname{x_{5}}{\left(t \right)} \right)}}{m}\\0\\0\end{matrix}\right], \
\mathbf{g}_2 = \left[\begin{matrix}0\\0\\0\\0\\0\\\frac{1}{J}\end{matrix}\right] -->
<p align="center">
  <img src="docs/plant_decom_eq.png" alt="image" width="60%" height="auto"/>
</p>

We employ ideas from nonlinear control theory and differential geometry to investigate the controllability of the plant.

**Theorem:** System $\dot{x}=f(x)+g(x)u$ with $x \in \mathbb{R}^n$ and $u \in \mathbb{R}^m$ is controllable if distribution $\Delta_{c2}$, the smallest distrribution spanned by columns of $g$, $span\{g_1, g_2\}$, and invariant to $g_1, g_2, f$, has rank $n$. 

Let's first instantiate a robot and get the symbolic system equations as follows:

```python
from DiffFlatQuad.robot import PlanerQuadrotor
robot = PlanerQuadrotor(rendering=False)

# Get the sympy symbolic expressions describing the plant
f = robot.getSymbolicF()
g = robot.getSymbolicG()
x = robot.symbolic_state
g1 = g[:,0]
g2 = g[:,1]
t = sp.symbols('t')
```
Then, use the following two helper functions to compute the Lie derivitives and brackets for the next step:

```python
def lieDerivative(a, b, x):
    """ 
    returns the L_a(b)
    """
    return b.jacobian(x)*a

def lieBracket(a,b,x):
    """
    returns the Lie bracket [a,b] = L_{a}b - L_{b}a
    """
    return lieDerivative(a, b, x)-lieDerivative(b, a, x)
```

**Reminder:** Distribution $\Delta$ is said to be invariant to be invariant to vector field $f$ if for all $\tau \in \Delta$, Lie bracket $[\tau, f]$ is also in $\Delta$.

To find the distribution $\Delta_{c2}$, we simply start with $\Delta = span\{g_1, g_2\}$ and compute the Lie brackets between $g1, g2$ and $f,g1,g2$ and add the result to the distribution if it's not already part of it. We continues this process untill the rank of $\Delta_{c2}$ stops growing:

<p align="center">
  <img src="docs/alg.png" alt="image" width="75%" height="auto"/>
</p>

The python implementation of this is:

```python
def isInDist(dist, vec):
    """
    Is vec in distribution dist?
    """
    d = dist[0].copy()
    for i in range(len(dist)-1):
        d = d.row_join(dist[i+1])
    rank1 = d.rank()
    d = d.row_join(vec)
    rank2 = d.rank()
    if rank2 > rank1:
        return False
    else:
        return True
    
def getDistRank(dist):
    """
    return the rank of distribution spanned by list of vecs in dist
    """
    d = dist[0].copy()
    for i in range(len(dist)-1):
        d = d.row_join(dist[i+1])
    return d.rank()

def getSmallestInvariantDistribution(dist, vec_fields, x):
    """
    Returns the smallest distribution invariant to vectors in the 
    vec_fields list and containing distribution spanned by the 
    vector fields in dist list. 
    """
    running = True
    result = []
    result +=dist
    while running:
        added_something = False
        for vec1 in result:
            for vec2 in vec_fields:
                vec3 = lieBracket(vec1, vec2, x)
                if not isInDist(result, vec3):
                    result +=[vec3]
                    added_something = True
        if not added_something:
            running = False
    
    return result
```

Using this funcitons, we can now compute the $\Delta_{c2}$ for our quadrotor plant. The result is showin in the folloiwng and has rank 6, which implies that the system is controllable.

<p align="center">
  <img src="docs/controllability_dist.png" alt="image" width="75%" height="auto"/>
</p>

## Controlling The Robot and Differential Flatness

Now that the cocntrollability of the system is shown, we can move on to designing a controller for the robot. We will first define differentially flat systems and then we will use it to design a controller. 

### Differentialy Flat Systems

System $\dot{x} = f(x,u)$, $x \in \mathbb{R}^n$ and $u \in \mathbb{R}^m$ is said to be differentially flat if there exists a function $z = \alpha(x,u, \dot{u}, ..., u^{(p)})$ from state and time derivitives of input to a set of outputs $z$ equal in number to the number of inputs such that we can reconstruct the whole state and input trajectory given the flat output $z$ and its time derivitives without integrating the dynamics:
<!-- 
x = &\beta(z,u, \dot{z}, ..., z^{(q)})\\
u = &\gamma(z,u, \dot{z}, ..., z^{(q)}) -->
<p align="center">
  <img src="docs/diff_flat.png" alt="image" width="25%" height="auto"/>
</p>

#### The Planar Quadrotor
For our planar quadrotor, the center of mass position $x_1,x_3$ are the flat outputs based upon which, the state and input trajectories may be computed as follows ([a great video showing this](https://www.youtube.com/watch?v=ohBJ0BCtZWQ)):

<!-- \left[\begin{matrix}\operatorname{{x}_{1}}{\left(t \right)}\\\operatorname{{x}_{2}}{\left(t \right)}\\\operatorname{{x}_{3}}{\left(t \right)}\\\operatorname{{x}_{4}}{\left(t \right)}\\\operatorname{{x}_{5}}{\left(t \right)}\\\operatorname{{x}_{6}}{\left(t \right)}\end{matrix}\right] = 
\left[\begin{matrix}\operatorname{x_{1}}{\left(t \right)}\\\frac{d}{d t} \operatorname{x_{1}}{\left(t \right)}\\\operatorname{x_{3}}{\left(t \right)}\\\frac{d}{d t} \operatorname{x_{3}}{\left(t \right)}\\\operatorname{atan_{2}}{\left(- \frac{d^{2}}{d t^{2}} \operatorname{x_{1}}{\left(t \right)},g + \frac{d^{2}}{d t^{2}} \operatorname{x_{3}}{\left(t \right)} \right)}\\- \frac{\left(g + \frac{d^{2}}{d t^{2}} \operatorname{x_{3}}{\left(t \right)}\right) \frac{d^{3}}{d t^{3}} \operatorname{x_{1}}{\left(t \right)}}{\left(g + \frac{d^{2}}{d t^{2}} \operatorname{x_{3}}{\left(t \right)}\right)^{2} + \left(\frac{d^{2}}{d t^{2}} \operatorname{x_{1}}{\left(t \right)}\right)^{2}} + \frac{\frac{d^{2}}{d t^{2}} \operatorname{x_{1}}{\left(t \right)} \frac{d^{3}}{d t^{3}} \operatorname{x_{3}}{\left(t \right)}}{\left(g + \frac{d^{2}}{d t^{2}} \operatorname{x_{3}}{\left(t \right)}\right)^{2} + \left(\frac{d^{2}}{d t^{2}} \operatorname{x_{1}}{\left(t \right)}\right)^{2}}\end{matrix}\right] -->

<p align="center">
  <img src="docs/diff_flat_state.png" alt="image" width="60%" height="auto"/>
</p>
<!-- \mathbf{u} = \left[\begin{matrix}m \sqrt{\left(g + \frac{d^{2}}{d t^{2}} \operatorname{x_{3}}{\left(t \right)}\right)^{2} + \left(\frac{d^{2}}{d t^{2}} \operatorname{x_{1}}{\left(t \right)}\right)^{2}}\\\frac{J \left(2 \left(\left(g + \frac{d^{2}}{d t^{2}} \operatorname{x_{3}}{\left(t \right)}\right) \frac{d^{3}}{d t^{3}} \operatorname{x_{1}}{\left(t \right)} - \frac{d^{2}}{d t^{2}} \operatorname{x_{1}}{\left(t \right)} \frac{d^{3}}{d t^{3}} \operatorname{x_{3}}{\left(t \right)}\right) \left(\left(g + \frac{d^{2}}{d t^{2}} \operatorname{x_{3}}{\left(t \right)}\right) \frac{d^{3}}{d t^{3}} \operatorname{x_{3}}{\left(t \right)} + \frac{d^{2}}{d t^{2}} \operatorname{x_{1}}{\left(t \right)} \frac{d^{3}}{d t^{3}} \operatorname{x_{1}}{\left(t \right)}\right) - \left(\left(g + \frac{d^{2}}{d t^{2}} \operatorname{x_{3}}{\left(t \right)}\right) \frac{d^{4}}{d t^{4}} \operatorname{x_{1}}{\left(t \right)} - \frac{d^{2}}{d t^{2}} \operatorname{x_{1}}{\left(t \right)} \frac{d^{4}}{d t^{4}} \operatorname{x_{3}}{\left(t \right)}\right) \left(\left(g + \frac{d^{2}}{d t^{2}} \operatorname{x_{3}}{\left(t \right)}\right)^{2} + \left(\frac{d^{2}}{d t^{2}} \operatorname{x_{1}}{\left(t \right)}\right)^{2}\right)\right)}{\left(\left(g + \frac{d^{2}}{d t^{2}} \operatorname{x_{3}}{\left(t \right)}\right)^{2} + \left(\frac{d^{2}}{d t^{2}} \operatorname{x_{1}}{\left(t \right)}\right)^{2}\right)^{2}}\end{matrix}\right] -->
<p align="center">
  <img src="docs/diff_flat_input.png" alt="image" width="100%" height="auto"/>
</p>

Note that the highet degree of flat output derivative is 4 which corresponds to the snap. Therefore, the designed desired trajectory must be smooth up to the 4'th order and to avoid actuation saturation, minimum-snap trajectories should be desingned. 

### Open-Loop Control
Given the mappings from the desired flat-output and the correspnding derivatives to the inputs and states, we can perform a simple test to see how the simulated quadrotor behaves when subjected to the computed input commands. Ideally, the drone should perfectly follow the desired trajectory. 

For the sake of simplicity, we chose a circular trajectory parametrized as $x=cos(\omega t), y=sin(\omega t)$ as reference and computed its derivitives up to the 4'th order (Look at `controller.ipynb` notebook). After applying the corresponding computed inputs to the simulated quadrotor, get the following response:

<p align="center">
  <img src="results/openloop_performance.png" alt="image" width="75%" height="auto"/>
</p>

As we can see, even though initially the drone stays close to the desired trajectory, after a short amount of time it deviates as there are no feedback loops to correct for the numerical inaccuracies (in this case forward Eular integration errors). To account for this, we need the magic of feedback!

### Close-Loop Control





