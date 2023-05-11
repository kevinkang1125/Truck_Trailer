 <h1 align = "center">RL Truck Trailer</h1>

#### Task Intro:

In this project, we are going to use deep reinforcement learning approach to teach the truck-trailer systems to conduct autonomous driving task on multi-scenarios, especially sharp turning. The agent model we adopt is show below：
<div>
    <center>
    <img src= './img/truck.png'
         alt= 'COM Gate'
         style='zoom:70%'>
    <br>
    </center>
</div>


For reinforcement learning, we need to find an appropriate simulation environment.
Now, we are using pybullet environment, it is easy to implement and coordinate well with reinforcement algorithm. The env we build in pybullet is shown below:

<div>
    <center>
    <img src= './img/env1.png'
         alt= 'COM Gate'
         style='zoom:70%'>
    <br>
    </center>
</div>

For now, we have a well defined state space(vector length = 7):
<div>
    <center>
    <img src= './img/state.png'
         alt= 'COM Gate'
         style='zoom:70%'>
    <br>
    </center>
</div>


And a simplified action space(vector length = 2):
<div>
    <center>
    <img src= './img/action.png'
         alt= 'COM Gate'
         style='zoom:70%'>
    <br>
    </center>
</div>


#### Current Stage:

We have finished the construction of gym environment of truck-trailer, the relative codes are stored in the **Truck-Trailer** folder. Now our training agent is the bus-like vehicle,which has simpler kinematic model than track-trailer system but share the same feature in length. Moreover, our current reward function is based on temporal difference of distance between vehicle and the target. A reward will be given with respect to the moving distance of the agent. Once the agent reach the target, it will receive a reward of 1000, and if it cannot reach the target within certain time, it will get a penalty of -1000. The function structure is shown below:
<div>
    <center>
    <img src= './img/reward.png'
         alt= 'COM Gate'
         style='zoom:70%'>
    <br>
    </center>
</div>

#### Agenda:

Our project will be conducted in 7 phases:\
step 1: construct Bus model\
step 2: finish the goal reaching task\
step 3: train Bus model in track environment\
step 4: construct Truck-Trailer model\
step 5: train Truck-Trailer model in track environment\
step 6: construct more complicated scenarios\
step 7: train Bus and Truck-Trailer model in the scenarios above

#### Requirements:
As details listed in requirements.txt

