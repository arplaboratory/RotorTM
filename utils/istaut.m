function flag = istaut(robot_pos,attach_pos,cable_length)
% istaut function identifies if the cable is taut
vecnorm(robot_pos - attach_pos,2,1);
flag = vecnorm(robot_pos - attach_pos,2,1) > cable_length - 1e-04; 

end