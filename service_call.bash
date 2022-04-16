rosservice call /traj_generator/Line "path:
- x: 0.0
  y: 0.0
  z: 0.5"

read -n 1 -s
rosservice call /traj_generator/Circle 1.0 10.0 10.0

read -n 1 -s                                   
rosservice call /traj_generator/Min_Derivative_Line "path:
- x: 0.5
  y: -0.5
  z: 0.25
- x: 1.0
  y: 0.0
  z: 0.5
- x: 1.5
  y: -0.5
  z: 0.75
- x: 2.0
  y: 0.0
  z: 1.0"
