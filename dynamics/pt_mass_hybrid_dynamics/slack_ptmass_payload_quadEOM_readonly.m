function sdot = slack_ptmass_payload_quadEOM_readonly(t, plqd, F, M, quad_params)
% QUADEOM_READONLY Solve quadrotor equation of motion
%   quadEOM_readonly calculate the derivative of the state vector
%
% INPUTS:
% t      - 1 x 1, time
% s      - 7 x 1, state vector = [qw, qx, qy, qz, pQuad, qQuad, rQuad]
% F      - 1 x 1, thrust output from controller (only used in simulation)
% M      - 3 x 1, moments output from controller (only used in simulation)
% params - struct, output from crazyflie() and whatever parameters you want to pass in
%
% OUTPUTS:
% sdotQuad   - 7 x 1, derivative of state vector s
%
% NOTE: You should not modify this function
% See Also: quadEOM_readonly, crazyflie

%************ EQUATIONS OF MOTION ************************

%Assign params and states
mQ = quad_params.mass;
e3=[0;0;1];
g = quad_params.grav*e3;
wRb=plqd.qd_rot;   %Rotation matrix of the quadrotor
qd_quat = plqd.qd_quat;
qd_omega = plqd.qd_omega;
p = qd_omega(1);
q = qd_omega(2);
r = qd_omega(3);

%Obtain Quadrotor Force Vector
quad_force_vector = F*wRb*e3;

%Solving for Quadrotor Acceleration
accQ = quad_force_vector/mQ - g; 

% Solving for Quadrotor Angular Velocity
K_quat = 2; %this enforces the magnitude 1 constraint for the quaternion
quaterror = 1 - norm(qd_quat);
qdot = -1/2*[0, -p, -q, -r;...
             p,  0, -r,  q;...
             q,  r,  0, -p;...
             r, -q,  p,  0] * qd_quat + K_quat*quaterror * qd_quat;

% Solving for Quadrotor Angular Acceleration
pqrdot   = quad_params.invI * (M - cross(qd_omega, quad_params.I*qd_omega));

% Assemble sdot
sdot = zeros(19,1);
sdot(1:3) = plqd.vel;
sdot(4:6) = -g;
sdot(7:9) = plqd.qd_vel;
sdot(10:12) = accQ;
sdot(13:16) = qdot;
sdot(17:19) = pqrdot;

end

