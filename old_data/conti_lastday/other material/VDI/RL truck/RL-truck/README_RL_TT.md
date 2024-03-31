 <h1 align = "center">RL Truck Trailer</h1>

#### Task Intro:

In this project, we are going to use deep reinforcement learning approach to teach the truck-trailer systems to conduct autonomous driving task on multi-scenarios, especially sharp turning. The agent model we adopt is show below：
<div>
    <center>
    <img src= './img/truck.png'
         alt= 'COM Gate'
         style='zoom:70%'>
    <br>
        Fig 1 Truck Parameters
    </center>
</div>


For reinforcement learning, we need to find an appropriate simulation environment. Originally, we plan to use the open source 3rd party package called **commonroad**, since it can provide us with rich library of road information and scenarios. However, the RL package they provide is not quite functional, and would take us a lot of time to do the debugging of the source code. In that case, we need to abandon it after one month testing, the codes we tried on commonroad environment is achieved in the  **cr_archieve** folder.
Now, we switch to pybullet environment, it is easy to implement and coordinate well with reinforcement algorithm. The env we build in pybullet is shown below:

<div>
    <center>
    <img src= './env1.png'
         alt= 'COM Gate'
         style='zoom:70%'>
    <br>
        Fig 2 Bus Vehicle in Pybullet
    </center>
</div>

For now, we have a well defined state space(vector length = 10):
$$
State Space=
\begin{bmatrix}
Coordinates&Orientation&Velocity&Distance2Goal&Deviation
\end{bmatrix}^T\\
Coordinates=
\begin{bmatrix}
x&y
\end{bmatrix}^T\\
Orientation=
\begin{bmatrix}
cos\psi&sin\psi
\end{bmatrix}^T\\
Velocity=
\begin{bmatrix}
v_x&v_y&\omega
\end{bmatrix}^T\\
Deviation=
\begin{bmatrix}
Rightwheel2Referenceline&Leftwheel2Referenceline
\end{bmatrix}^T\\
$$

state v2:
$$
State Space=
\begin{bmatrix}
Orientation&Velocity&Distance2Goal&LocalGoal
\end{bmatrix}^T\\
Orientation=
\begin{bmatrix}
cos\psi&sin\psi
\end{bmatrix}^T\\
Velocity=
\begin{bmatrix}
v_x&v_y&\omega
\end{bmatrix}^T\\
LocalGoal=
\begin{bmatrix}
X_{local}&Y_{local}
\end{bmatrix}^T\\
X_{local}=(X_{target}-X_{vehicle})*cos\psi+(Y_{target}-Y_{vehicle})*sin\psi\\
Y_{local}=-(X_{target}-X_{vehicle})*sin\psi+(Y_{target}-Y_{vehicle})*cos\psi\\
$$




$$
State Space=
\begin{bmatrix}
Coordinates&Orientation&Velocity&Goal
\end{bmatrix}^T\\
Coordinates=
\begin{bmatrix}
X_{vehicle}&Y_{vehicle}
\end{bmatrix}^T\\
Orientation=
\begin{bmatrix}
cos\psi&sin\psi
\end{bmatrix}^T\\
Velocity=
\begin{bmatrix}
v_x&v_y&\omega
\end{bmatrix}^T\\
Goals=
\begin{bmatrix}
X_{target}&Y_{target}
\end{bmatrix}^T\\
$$


And a simplified action space(vector length = 2):
$$
Action Space=
\begin{bmatrix}
Throttle&Steer
\end{bmatrix}^T\\
$$

reward function:
$$
reward = r_{range}+r_{reach}+r_{proceed}\\
$$

$$
r_{range}=
\begin{cases}
2 & \text{if min(Rightwheel2Referenceline,Leftwheel2Referenceline)}\leq2.5m\\
-2& \text{if min(Rightwheel2Referenceline,Leftwheel2Referenceline)}>2.5m\\
-10 & \text{if min(Rightwheel2Referenceline,Leftwheel2Referenceline)}>5m
\end{cases}\\
$$

$$
r_{reach}=
\begin{cases}
10 & \text{if Distance2Goal}\leq5m\\
0& \text{if Distance2Goal}>5m\\
\end{cases}\\
$$

$$
r_{proceed}=
\begin{cases}
2 & \text{if current Distance2Goal}\leq\text{former Distance2Goal}\\
-1& \text{if current Distance2Goal}>\text{former Distance2Goal}\\
\end{cases}\\
$$

$$
State Space=
\begin{bmatrix}
Orientation&Velocity&Waypoints&Hitch
\end{bmatrix}^T\\
Orientation=
\begin{bmatrix}
cos\psi&sin\psi
\end{bmatrix}^T\\
Velocity=
\begin{bmatrix}
v_x&v_y&\omega
\end{bmatrix}^T\\
Waypoints=
\begin{bmatrix}
Dist_1&X_{1_{local}}&Y_{2_{local}}&Dist_2&X_{2_{local}}&Y_{2_{local}}
\end{bmatrix}^T\\
X_{local}=(X_{target}-X_{vehicle})*cos\psi+(Y_{target}-Y_{vehicle})*sin\psi\\
Y_{local}=-(X_{target}-X_{vehicle})*sin\psi+(Y_{target}-Y_{vehicle})*cos\psi\\
$$

$$
State Space=
\begin{bmatrix}
Orientation&Velocity&Waypoints
\end{bmatrix}^T\\
Orientation=
\begin{bmatrix}
cos\psi&sin\psi
\end{bmatrix}^T\\
Velocity=
\begin{bmatrix}
v_x&v_y&\omega
\end{bmatrix}^T\\
Waypoints=
\begin{bmatrix}
Dist_1&X_{1_{local}}&Y_{2_{local}}&Dist_2&X_{2_{local}}&Y_{2_{local}}
\end{bmatrix}^T\\
X_{local}=(X_{target}-X_{vehicle})*cos\psi+(Y_{target}-Y_{vehicle})*sin\psi\\
Y_{local}=-(X_{target}-X_{vehicle})*sin\psi+(Y_{target}-Y_{vehicle})*cos\psi\\
$$

reward function v2
$$
reward = r_{reach}+r_{proceed}+r_{time out}\\
$$


$$
r_{threshold}
\begin{cases}
10 & \text{if dist2target}\leq180m\\
20& \text{if dist2target}\leq160m\\
40 & \text{if dist2target}\leq120m\\
60 & \text{if dist2target}\leq80m\\
80 & \text{if dist2target}\leq40m\\
\end{cases}\\
$$

$$
r_{reach}=100
$$

$$
r_{proceed}=-\frac{210}{230-dist2target}
$$

$$
r_{reach}=100
$$

reward v3
$$
reward = r_{reach}+r_{proceed}\\
$$

$$
r_{proceed}=\frac{1}{dist2target}
$$

$$
r_{reach}=100
$$



reward v4
$$
reward = r_{reach}+r_{proceed}+r_{time out}\\
$$

$$
r_{proceed}=({Dist2Target_{t-1}-Dist2target_t})*K
$$

$$
r_{reach}=1000
$$

$$
r_{timeout}=-1000
$$



#### Track Env :

reward v1:
$$
reward = r_{reach}+r_{proceed}+r_{time out}+r_{deviation}\\
r_{reach}=
\begin{cases}
100 & \text{if Distance2NearTarget}\leq1.5m\\
0& \text{if Distance2NearTarget}>1.5m\\
\end{cases}\\
r_{time out}=
\begin{cases}
0 & \text{if Timesteps}\leq60000\\
-1000& \text{if Timesteps}>60000\\
\end{cases}\\
r_{time out}=
\begin{cases}
0 & \text{if Lateral Deviation}\leq10\\
-1000& \text{if Lateral Deviation}>10\\
\end{cases}\\
r_{proceed}=({Dist2NearTarget_{t-1}-Dist2NearTarget_t})*K
$$

reward v2:
$$
reward = r_{reach}+r_{proceed}+r_{time out}+r_{deviation}\\
r_{reach}=
\begin{cases}
10 & \text{if Distance2NearTarget}\leq1.5m\\
0& \text{if Distance2NearTarget}>1.5m\\
\end{cases}\\
r_{time out}=
\begin{cases}
0 & \text{if Timesteps}<25000\\
-100& \text{if Timesteps}=25000\\

\end{cases}\\
r_{deviation}=
\begin{cases}
0 & \text{if Lateral Deviation}\leq10\\
-100& \text{if Lateral Deviation}>10\\
\end{cases}\\
r_{proceed}=({Dist2NearTarget_{t-1}-Dist2NearTarget_t})*K &K=1500/TrackLength
$$

reward v3
$$
reward = r_{reach}+r_{proceed}+r_{time out}+r_{deviation}+r_{out}\\
r_{reach}=
\begin{cases}
10 & \text{if Distance2NearTarget}\leq1.5m\\
0& \text{if Distance2NearTarget}>1.5m\\
\end{cases}\\
r_{time out}=
\begin{cases}
0 & \text{if Timesteps}<25000\\
-100& \text{if Timesteps}=25000\\

\end{cases}\\
r_{out}=
\begin{cases}
0 & \text{if Lateral Deviation}\leq10\\
-100& \text{if Lateral Deviation}>10\\
\end{cases}\\
r_{deviation}=- \text{Lateral Deviation}^2*V &V=0.005\\
r_{proceed}=({Dist2NearTarget_{t-1}-Dist2NearTarget_t})*K &K=1500/TrackLength
$$

$$
reward = r_{reach}+r_{proceed}+r_{time out}+r_{deviation}\\
r_{reach}=
\begin{cases}
10 & \text{if Distance2NearTarget}\leq1.5m\\
0& \text{if Distance2NearTarget}>1.5m\\
\end{cases}\\
r_{time out}=
\begin{cases}
0 & \text{if Timesteps}<25000\\
-100& \text{if Timesteps}=25000\\

\end{cases}\\
r_{deviation}=-({d_1}^2+wd_2^2)*V &V=0.005\\
r_{proceed}=({Dist2NearTarget_{t-1}-Dist2NearTarget_t})*K &K=1500/TrackLength
$$



$$
reward = r_{speed}+r_{time out}+r_{deviation}\\
r_{time out}=
\begin{cases}
0 & \text{if Timesteps}<25000\\
-100& \text{if Timesteps}=25000\\

\end{cases}\\
r_{deviation}=-({d_1}^2+wd_2^2)*D &D=0.005\\
r_{speed}=-(V_{real}-V_{target})^2*S &S=0.01
$$

$$
reward = r_{reach}+r_{proceed}+r_{timeout}+r_\text{off-track}\\
r_{reach}=
\begin{cases}
40 & \text{if Distance2NearTarget}\leq1.5m\\
0& \text{if Distance2NearTarget}>1.5m\\
\end{cases}\\
r_{time out}=
\begin{cases}
0 & \text{if Timesteps}<25000\\
-400& \text{if Timesteps}=25000\\

\end{cases}\\
r_\text{off-track}=-200\\
r_{proceed}=({Dist2NearTarget_{t-1}-Dist2NearTarget_t})*K &K=6000/TrackLength
$$

#### Current Stage:

##### We have finished the construction of gym environment of truck-trailer, the relative codes are stored in the **Truck-Trailer** folder. Now our training agent is the bus-like vehicle,which has simpler kinematic model than track-trailer system but share the same feature in length. Moreover, our current reward function is based on the deviation from the reference path, the closer the vehicle get to the reference line, the more reward it will get,vice versa.

#### Agenda:

Our project will be conducted in 7 phases:
step 1: construct Bus model
step 2: construct scenarios for relatively straight line
step 3: train bus model in straight line scenarios
step 4: construct Truck-Trailer model
step 5: train Truck-Trailer model in straight line scenarios
step 6: construct more complicated scenarios

##### step 7: train Bus and Truck-Trailer model in the scenarios above

#### Requirements:

##### reward
