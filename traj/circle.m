function [state_struct,stop] = circle(t, state_struct, init_pos, r, period, circle_duration)

% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
persistent Radius;
persistent ramp_theta_coeff;
persistent zcoeff;
persistent tf;
persistent last_pos;
persistent offset_pos;
persistent T;
persistent omega_des;
persistent ramp_t;
persistent ramp_dist;
persistent circle_dist;
persistent start
persistent duration;

if nargin~=2
    disp('Generating Circular Trajectory ...');
    Radius = r;
    offset_pos = [init_pos(1)-Radius;init_pos(2);0];
    T = period;
    omega_des = 2*pi/T;
    
    alpha_des = pi/40;
    start = init_pos;
    ramp_t = omega_des/alpha_des;
    duration = circle_duration;
    
    %zinitial = [0 Radius/2 0 0 0 0]';
    %zinitial = [init_pos(3) 0 0 end_pos(3) 0 0]';
    %thetainitial = [0 2*pi 0 0 0 0]';
    thetainitial = [0 0 0 ramp_t*omega_des 0 0]';
    A = [generate_poly(5,2,0);
        generate_poly(5,2,1)];
    
    %A = [1  t0 t0^2 t0^3   t0^4   t0^5;
    %     1  tf tf^2 tf^3   tf^4   tf^5;
    %     0  1  2*t0 3*t0^2 4*t0^3 5*t0^4;
    %     0  1  2*tf 3*tf^2 4*tf^3 5*tf^4;
    %     0  0  2    6*t0   12*t0^2 20*t0^3;
    %     0  0  2    6*tf   12*tf^2 20*tf^3;];
    
    %zcoeff = inv(A)*zinitial;
    ramp_theta_coeff = inv(A)*thetainitial;
    ramp_dist = sum(ramp_theta_coeff.*[1,1/2,1/3,1/4,1/5,1/6]');
    circle_dist = omega_des*duration;
    tf = ramp_t * 2 + duration;
    final_theta = 2*ramp_dist + circle_dist;
    x_pos = Radius * cos(final_theta);
    y_pos = Radius * sin(final_theta);
    stop = offset_pos + [x_pos ; y_pos;start(3)];
    
else
    
    %polynominalmat = [1 t  (t)^2   (t)^3   (t)^4   (t)^5;
    %                  0 1  2*(t)   3*(t)^2 4*(t)^3 5*(t)^4;
    %                  0 0  2       6*(t)   12*(t)^2 20*(t)^3];
    if t < tf
        
        if t<=ramp_t % ramping up the circle
            dt = t/ramp_t;
            integral_poly = generate_poly(6,0,dt);
            integral_poly = integral_poly(2:7).*[1,1/2,1/3,1/4,1/5,1/6];
            polynominalmat = [integral_poly;generate_poly(5,2,dt)];
            
            theta_d = polynominalmat*ramp_theta_coeff;
            theta_d = theta_d.*[1;1/ramp_t;1/ramp_t^2;1/ramp_t^3];
            %z_d = polynominalmat*zcoeff;
            
        elseif (t <= ramp_t + duration) % constant velocity cruising
            dt = t - ramp_t;
            theta_d = zeros(4,1);
            theta_d(1) = omega_des * dt + ramp_dist;
            theta_d(2) = omega_des;
            
        else  % ramping down the circle
            dt = 1 - (t - duration - ramp_t)/ramp_t;
            integral_poly = generate_poly(6,0,dt);
            integral_poly = integral_poly(2:7).*[1,1/2,1/3,1/4,1/5,1/6];
            polynominalmat = [integral_poly;generate_poly(5,2,dt)];
            
            theta_d = polynominalmat*ramp_theta_coeff;
            theta_d = theta_d.*[1;1/ramp_t;1/ramp_t^2;1/ramp_t^3];
            theta_d(1) = circle_dist + 2*ramp_dist - theta_d(1);
        end
        
        x_pos = Radius * cos(theta_d(1));
        y_pos = Radius * sin(theta_d(1));
        x_vel = -Radius * sin(theta_d(1)) * theta_d(2);
        y_vel =  Radius * cos(theta_d(1)) * theta_d(2);
        x_acc = -Radius * cos(theta_d(1)) * theta_d(2)^2 - Radius * sin(theta_d(1)) * theta_d(3);
        y_acc = -Radius * sin(theta_d(1)) * theta_d(2)^2 + Radius * cos(theta_d(1)) * theta_d(3);
        x_jrk = Radius * sin(theta_d(1)) * theta_d(2)^3 - 3 * Radius * cos(theta_d(1)) * theta_d(2) * theta_d(3) - Radius * sin(theta_d(1)) * theta_d(4);
        y_jrk = -Radius * cos(theta_d(1)) * theta_d(2)^3 - 3 * Radius * sin(theta_d(1)) * theta_d(2) * theta_d(3) + Radius * cos(theta_d(1)) * theta_d(4);
        
        pos = offset_pos + [x_pos ; y_pos;start(3)];
        last_pos = pos;
        vel = [x_vel ; y_vel; 0];
        acc = [x_acc ; y_acc; 0];
        jrk = [x_jrk ; y_jrk; 0];
        yaw = 0;
        yawdot = 0;
        
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
