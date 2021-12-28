classdef CablePlot < handle
    %QUADPLOT Visualization class for quad

    properties (SetAccess = public)
        k = 0;
        qn;             % quad number
        time = 0;       % time
        state;          % state
        cable;          % cable start and cable end
        des_state;      % desried state [x; y; z; xdot; ydot; zdot]
        rot;            % rotation matrix body to world

        color;          % color of quad
        len;            % cable length
        height;         % height of quad
        motor;          % motor position

        state_hist        % position history
        state_des_hist;   % desired position history
        time_hist;      % time history
        max_iter;       % max iteration
    end

    properties (SetAccess = private)
        h_3d
        h_cb;  % cable state plot handle
        h_pos_hist;     % position history handle
        h_pos_des_hist; % desired position history handle
        text_dist;  % distance of quad number to quad
    end

    methods
        % Constructor
        function Q = CablePlot(qn, state, attach, len, color, max_iter, h_3d)
            Q.qn = qn;
            Q.state = state;
            Q.color = color;
            Q.len = len;
            Q.cable = [attach, -Q.state(1:3)*Q.len + attach];

            Q.max_iter = max_iter;
            Q.state_hist = zeros(6, max_iter);
%             Q.state_des_hist = zeros(6, max_iter);
            Q.time_hist = zeros(1, max_iter);

            % Initialize plot handle
            if nargin < 7, h_3d = gca; end
            Q.h_3d = h_3d;
            hold(Q.h_3d, 'on')
            Q.h_cb = plot3(Q.h_3d, ...
                Q.cable(1,:), ...
                Q.cable(2,:), ...
                Q.cable(3,:), ...
                'Color', Q.color);
            hold(Q.h_3d, 'off')
        end

        % Update quad state
        function UpdateCableState(Q, state, attach_pos, time)
            Q.state = state;
            Q.cable = [attach_pos, -Q.state(1:3)*Q.len + attach_pos];
            Q.time = time;
        end

        % Update desired quad state
        function UpdateDesiredCableState(Q, des_state)
            Q.des_state = des_state;
        end

        % Update quad history
        function UpdateCableHist(Q)
            Q.k = Q.k + 1;
            Q.time_hist(Q.k) = Q.time;
            Q.state_hist(:,Q.k) = Q.state;
%             Q.state_des_hist(:,Q.k) = Q.des_state(1:6);
        end

        % Truncate history
        function TruncateHist(Q)
            Q.time_hist = Q.time_hist(1:Q.k);
            Q.state_hist = Q.state_hist(:, 1:Q.k);
%             Q.state_des_hist = Q.state_des_hist(:, 1:Q.k);
        end

        % Update quad plot
        function UpdateCablePlot(Q, state, attach_pos, time)
            Q.UpdateCableState(state, attach_pos, time);
%             Q.UpdateDesiredQuadState(des_state);
            Q.UpdateCableHist();
            set(Q.h_cb, ...
                'XData', Q.cable(1,:), ...
                'YData', Q.cable(2,:), ...
                'ZData', Q.cable(3,:));
            drawnow;
        end
    end

end
