# RotorTM
An Aerial Transportation and Manipulation Simulator for Research and Education

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
The RotorTM package is dependent on MATLAB(>version 2020), MATLAB ROS Toolbox and ROS. 
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
By default, the following parameter files are chosen: 
|Name|Chosen Files|
|---|---|
|`config/uav_params/snapdragonfly.yaml`|phd or jpdaf specify in the demo.launch file|
|`config/control_params/dragonfly_control_gains.yaml`|output of py_imag_proc or /hummingbird0/track/bounding_box specify in the demo.launch file|
|`config/load_params/triangular_payload.yaml`|/hires/image_raw or hummingbird0/camera/rgb specify in the demo.launch file|
|`config/control_params/triangular_payload_cooperative_cable_gains.yaml`|output of dragonfly imu or hummingbird0/imu specify in the demo.launch file|
|`config/attach_mechanism/3_robots_cable_mechanism.yaml`|how many drones in FOV specify in the demo.launch file|
|`config/initial_condition/3_robots_triangular_payload_initial_condition.yaml`|how many drones in FOV specify in the demo.launch file|

```
$ roslaunch RotorTM rviz.launch
```
