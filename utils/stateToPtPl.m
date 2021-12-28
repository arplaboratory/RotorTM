function [qd] = stateToPtPl(x)
%Converts qd struct used in hardware to x vector used in simulation
% x is 1 x 6 vector of state variables [pos vel]
% qd is a struct including just the pos and vel if the payload is a point
%    mass

%current state
qd.pos = x(1:3);
qd.vel = x(4:6);

end
