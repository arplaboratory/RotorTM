function [state_struct,stop] = offline_traj(t, state_struct, traj_path)

% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables


persistent tf;
persistent offline_traj;
persistent last_pos;

if nargin~=2

    load(traj_path);
    offline_traj = robot_cmd;
    tf = offline_traj{1}.time(end);
    
else    

    if t < tf
         
        [~, idx] = min(abs(t - offline_traj{1}.time));
        pos = [offline_traj{1}.x(idx) ; offline_traj{1}.y(idx); offline_traj{1}.z(idx)];
        vel = [offline_traj{2}.x(idx) ; offline_traj{2}.y(idx); offline_traj{2}.z(idx)];
        acc = [offline_traj{3}.x(idx) ; offline_traj{3}.y(idx); offline_traj{3}.z(idx)];
        jrk = [offline_traj{4}.x(idx) ; offline_traj{4}.y(idx); offline_traj{4}.z(idx)];
        yaw = 0;
        yawdot = 0;

        last_pos = pos;
        
    else
        
        pos = last_pos;
        vel = [0 ; 0; 0];
        acc = [0 ; 0; 0];
        jrk = [0 ; 0; 0];
        yaw = 0;
        yawdot = 0;
    
    end
   
    state_struct.pos_des = pos(:);
    state_struct.vel_des = vel(:);
    state_struct.acc_des = acc(:);
    state_struct.jrk_des = jrk(:);
    state_struct.qd_yaw_des = yaw;
    state_struct.qd_yawdot_des = yawdot;
    state_struct.quat_des = [1,0,0,0];
    state_struct.omega_des = [0,0,0];
    
end
