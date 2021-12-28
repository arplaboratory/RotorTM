function [F,M] = clamp_thrust_moment(F,M,params)
% This functio clamp the thrust and moment based on the min thrust and max
% thrust of the quadrotor

% Limit the force and moments due to actuator limits
A = [0.25,                      0, -0.5/params.arm_length;
     0.25,  0.5/params.arm_length,                      0;
     0.25,                      0,  0.5/params.arm_length;
     0.25, -0.5/params.arm_length,                      0];
 
prop_thrusts = A*[F;M(1:2)]; % Not using moment about Z-axis for limits
prop_thrusts_clamped = max(min(prop_thrusts, params.maxF/4), params.minF/4);

B = [                 1,                 1,                 1,                  1;
                      0, params.arm_length,                 0, -params.arm_length;
     -params.arm_length,                 0, params.arm_length,                 0];

F = B(1,:)*prop_thrusts_clamped;
M = [B(2:3,:)*prop_thrusts_clamped; M(3)];

end