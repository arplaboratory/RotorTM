function sdot = hybrid_cooperative_rigidbody_pl_transportationEOM(t, s, nquad, plcontrolhdl, trajhandle, quad_params, pl_params)
% QUADEOM Wrapper function for solving quadrotor equation of motion
% 	quadEOM takes in time, state vector, controller, trajectory generator
% 	and parameters and output the derivative of the state vector, the
% 	actual calcution is done in quadEOM_readonly.
%
% INPUTS:
% t             - 1 x 1, time
% s             - (7*nquad + 6*nquad + 13) x 1,
%                 state vector = [xL, yL, zL, xLd, yLd, zLd,
%                                 qLw, qLx, qLy, qLz, pL, qL, rL,
%                                 [xQ, yQ, zQ, xQd, yQd, zQd]_i, i = 1,...,nquad
%                                 [qw, qx, qy, qz, pQuad, qQuad, rQuad]_i, i = 1,...,nquad
% nquad         - number of quads
% qn            - quad id (used for multi-robot simulations)
% plcontrolhdl  - function handle of payload controller
% qdcontrolhdl  - function handle of quad attitude controller
% trajhandle    - function handle of payload trajectory generator
% params        - struct, output from crazyflie() and whatever parameters you want to pass in

% OUTPUTS:
% sdot          - 13*(nquad+1) x 1, derivative of state vector s

global cable_is_slack;
global sim_start;
persistent pl_accel;
persistent pl_ang_accel;
persistent attach_accel;

if ~sim_start
    pl_accel = zeros(3,1);
    pl_ang_accel = zeros(3,1);
    attach_accel = zeros(3,1);
    sim_start = true;
end

%[pl, qd] = stateToPlQdTeamCable(s,pl_params,quad_params);

% convert state to payload stuct for control
pl_dim_num = 13;
quad_dim_num = 13;
rho_vec_list = pl_params.rho_vec_list;
cable_len_list = pl_params.cable_length;

pl_state = s(1:pl_dim_num);
pl = stateToPl(pl_state);
pl_rot = pl.rot; % The payload rotation matrix w.r.t world frame.
pl_omega_asym = vec2asym(pl.omega);
qd_state = reshape(s(pl_dim_num + 1 : end),quad_dim_num,nquad);
qd_pos = qd_state(1:3,:);
qd_vel = qd_state(4:6,:);

robot_attach_vector = pl.pos + pl_rot*rho_vec_list - qd_pos;
qd_xi = robot_attach_vector./vecnorm(robot_attach_vector,2,1);
qd_xidot = (pl.vel + pl_rot * pl_omega_asym * rho_vec_list - qd_vel)./cable_len_list;
cable_omg = cross(qd_xi,qd_xidot);

% Get desired_state for the payload
pl = trajhandle(t,pl); %TODO: remove qn id in the trajhandle since this is for payload, or you don't need to?


%% Dynamics Parameters
ML = pl_params.mass * eye(3);
% ML = 1000 * eye(3);
C = zeros(3,3);
D = zeros(3,3);
E = zeros(3,3);
pl_net_F = zeros(3,1);
pl_net_M = zeros(3,1);
attach_accel = pl_accel + quad_params.grav*[0;0;1] + pl_rot*vec2asym(pl_ang_accel)*rho_vec_list + pl_rot*pl_omega_asym*pl_omega_asym*rho_vec_list;

for qn = 1:nquad
    
    %% Assignment of Quadrotor's State and Cable's State
    qd{qn} = stateToQd(qd_state(:,qn));
    qd{qn}.xi = qd_xi(:,qn);
    qd{qn}.xixiT = qd_xi(:,qn)*qd_xi(:,qn)';
    qd{qn}.xidot = qd_xidot(:,qn);
    rho_qn_asym = pl_params.rho_vec_asym_mat(:,3*qn-2:3*qn);
    
    omega_qn = cable_omg(:,qn);
    
    %qd{qn}.attach_accel = pl_accel + quad_params.grav*[0;0;1] - pl.rot*rho_qn_asym*pl_ang_accel + pl.rot*attach_centrifugal_accel;
    %qd{qn}.attach_accel = attach_accel(:,qn);
    quad_params.l = cable_len_list(qn);
    quad_mass_diagonal = quad_params.mass * eye(3);
    
    % TODO: the planning for yaw, yawdot of quad
    qd{qn}.yaw_des = 0;
    qd{qn}.yawdot_des = 0;
    if ~cable_is_slack(qn)
        Ck =  quad_mass_diagonal * rho_qn_asym * pl_rot' * qd{qn}.xixiT;
        Dk = -Ck';
        Ek = Ck * pl_rot * rho_qn_asym;
        C = C + Ck;
        D = D + Dk;
        E = E + Ek;
        ML = ML + quad_mass_diagonal * qd{qn}.xixiT;
    end
    
end

pl.C = C;
pl.D = D;
pl.E = E;
pl.invML = inv(ML);

%% Payload Controller 
[F_list,M_list] = plcontrolhdl(pl,qd,pl_params,quad_params);

for qn = 1:nquad

    % Quadrotor Attitude Controller to track desired cable directions
    % TODO: take robot position, payload postion
    % robot velocity and payload velocity as input.
    %[F, M, trpy, drpy] = qdattcontrolhdl(qd, qn, quad_params);
    % Clamp controller output based on quadrotor's limit
    [F, M] = clamp_thrust_moment(F_list{qn}, M_list{qn}, quad_params);
    qd{qn}.F = F;
    qd{qn}.M = M;        
    qd_u = qd{qn}.rot*[0;0;F];
    u(:,qn) = qd_u;
    u_parallel = qd{qn}.xixiT * qd_u;
    
    if ~cable_is_slack(qn)
        rho_qn_asym = pl_params.rho_vec_asym_mat(:,3*qn-2:3*qn);
        omega_qn = cable_omg(:,qn);
        attach_centrifugal_accel = pl_omega_asym * pl_omega_asym * rho_vec_list(:,qn);
        % If the cable is taut, calculate these terms
        % Force and Moment at attach point
        attach_qn_force = u_parallel ...
            - quad_params.mass * cable_len_list(qn) * (omega_qn' * omega_qn) * qd{qn}.xi ...
            - quad_mass_diagonal * qd{qn}.xixiT * pl_rot * attach_centrifugal_accel;
        attach_qn_moment = rho_qn_asym * pl_rot' * attach_qn_force;
        
        % Sum Net Force, Moment and other corresponding terms
        % for the Payload Dynamics
        pl_net_F = pl_net_F + attach_qn_force;
        pl_net_M = pl_net_M + attach_qn_moment;
    end
    
end

%% Dynamics of Payload

sdotLoad = rigidbody_payloadEOM_readonly(t, pl, pl_net_F, pl_net_M, pl_params); %Payload Eom read only
pl_accel = sdotLoad(4:6);%zeros(3,1);%
pl_ang_accel = sdotLoad(11:13);%zeros(3,1);%
sdot = sdotLoad;

%if pl_params.sim_start
attach_accel = pl_accel + quad_params.grav*[0;0;1] + pl_rot*vec2asym(pl_ang_accel)*rho_vec_list + pl_rot*pl_omega_asym*pl_omega_asym*rho_vec_list;
%end

%% Dynamics of Quadrotors (Hybrid)
% Tk = mk*lk * norm(xidot)^2 - xi' * (uk(qd_u) - mk*attach_accel_k)
T = quad_params.mass.* pl_params.cable_length.*sum(cable_omg.*cable_omg,1) - diag(qd_xi'*(u-quad_params.mass*attach_accel))';

for qn = 1:nquad
    if cable_is_slack(qn)
        sdotQuad = slack_quadEOM_readonly(t,qd{qn},quad_params);
    else
        qd{qn}.T = T(qn);
        sdotQuad = taut_quadEOM_readonly(t,qd{qn},quad_params);
    end
    sdot = [sdot; sdotQuad];
end

end
