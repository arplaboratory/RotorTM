function [qd] = stateToPl(x)
%Converts qd struct used in hardware to x vector used in simulation
% x is 1 x 13 vector of state variables [pos vel quat omega]
% qd is a struct including the fields pos, vel, euler, and omega

%current state
qd.pos = x(1:3);
qd.vel = x(4:6);
qd.quat = x(7:10);
qd.omega = x(11:13);
qd.rot = QuatToRot(qd.quat)';

end
