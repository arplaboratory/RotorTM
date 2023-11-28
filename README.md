# RotorTM
An Aerial Transportation and Manipulation Simulator for Research and Education

paper: https://arxiv.org/abs/2205.05140

video: https://www.youtube.com/watch?v=jzfEVQ3qlPc

## License
Please be aware that this code was originally implemented for research purposes and may be subject to changes and any fitness for a particular purpose is disclaimed. To inquire about commercial licenses, please contact Guanrui Li (lguanrui@nyu.edu), Xinyang Liu (liuxy@nyu.edu) and Prof. Giuseppe Loianno (loiannog@nyu.edu).
```
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
    
```
## Citation
If you publish a paper with our simulator, please cite our paper published in IEEE Transactions on Robotics: 
```
@ARTICLE{rotortm2023,
  author={Li, Guanrui and Liu, Xinyang and Loianno, Giuseppe},
  journal={IEEE Transactions on Robotics}, 
  title={RotorTM: A Flexible Simulator for Aerial Transportation and Manipulation}, 
  year={2023},
  volume={},
  number={},
  pages={1-20},
  doi={10.1109/TRO.2023.3336320}}
 ```

## Overview
#### Description
RotorTM is an aerial transportation and manipulation simulator of MAVs with different payloads and passive connection mechanisms. It incorporates full system
dynamics as well as planning, and control algorithms for aerial transportation and manipulation. Furthermore, it includes a hybrid model accounting for the transient hybrid dynamics for aerial systems with cable suspended load to mimic real-world systems. It also provides flexible interfaces to planning and control software modules for the users. 

If you have any question regarding the repo or how to use the simulator please feel free to post questions in the Issues. 

![Screenshot](doc/intro.png)

**Developer: Guanrui Li<br />
Affiliation: [NYU ARPL](https://wp.nyu.edu/arpl/)<br />
Maintainer: Guanrui Li, lguanrui@nyu.edu<br />**

#### Published Topics
|Name|Type|Description|
|---|---|---|
|`/system/marker`|visualization_msgs/MarkerArray|Visualization of the entire system, including the robots, payload, and cables.|
|`/payload/path`|visualization_msgs/Marker|Visualization of the payload position path|
|`/payload/des_path`|visualization_msgs/Marker|Visualization of the payload desired postion path|

#### Parameters Files
|Name|Description|
|---|---|
|`UAV Params`|Basic UAV parameters like |
|`Payload Params`|Basic payload parameters like mass, moment of inertia etc.|
|`UAV Controller Params`|UAV controller parameters|
|`Payload Controller Params`|Payload controller parameters|
|`Attach Mechanism Params`|Attach mecahnism parameters|
|`Initial Condition`|Initial conditions for robots and payload|

## Dependencies and Installation
The RotorTM package is dependent on MATLAB(>version 2020), MATLAB ROS Toolbox, MATLAB Optimization Tookbox, and ROS. 
```
$ cd /path/to/your/workspace
$ git clone https://github.com/arplaboratory/RotorTM.git
$ cd RotorTM
$ catkin build RotorTM
```

## Running

#### Use GUI Interface
In your MATLAB, run the RotorTM GUI app like the following

```
>> RotorTM_GUI
```
Then you should be able to see the following GUI interface:

<img src="https://github.com/arplaboratory/RotorTM/blob/main/doc/gui.png" width="450" height="300">

The users can use the interface to choose different parameter files by clicking the corresponding buttion. 
By default, the following parameter files are chosen to simulate 3 quadrotors transporting a triangular payload: 
|Name|Chosen Files|
|---|---|
|`UAV Params`|`config/uav_params/snapdragonfly.yaml`|
|`Payload Params`|`config/load_params/triangular_payload.yaml`|
|`UAV Controller Params`|`config/control_params/dragonfly_control_gains.yaml`|
|`Payload Controller Params`|`config/control_params/triangular_payload_cooperative_cable_gains.yaml`|
|`Attach Mechanism Params`|`config/attach_mechanism/3_robots_cable_mechanism.yaml`|
|`Initial Condition`|`config/initial_condition/3_robots_triangular_payload_initial_condition.yaml`|

You can check in [here](https://github.com/arplaboratory/RotorTM/blob/main/doc/Simulator_Params.md) to see different kinds of combinatioins of parameters to simulate different situations. Then you can choose the type of trajectory generator to generate trajectory for the payload. We provide 
1. circular trajectory generator
2. minimum derivative trajectory generator

After you choose the trajectory generator type, you can click the `Setup Parameters` button. 
It will pop out one of the two following windows depending on the trajectory generator type you choose: 
1. <img src="https://github.com/arplaboratory/RotorTM/blob/main/doc/circle.png" width="500" height="125"> 
2. <img src="https://github.com/arplaboratory/RotorTM/blob/main/doc/min_derivative.png" width="500" height="225">
You can choose your own trajectory parameters, which you can see [here](https://github.com/arplaboratory/RotorTM/blob/main/doc/trajectory_generator.md) for further explanation of the parameters. 

After you finish choosing the parameters for trajectory generator, you can click the `Set` button. Then you can hit the `Run` button in the main GUI and the simulation should be running. You should be able to see the visualization by running the following command in the terminal window: 
```
$ roslaunch RotorTM rviz.launch
```
