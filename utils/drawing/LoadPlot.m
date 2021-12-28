classdef LoadPlot < handle
    %LoadPLOT Visualization class for the rigid body payload

    properties (SetAccess = public)
        k = 0;
        time = 0;       % time
        state;          % state
        des_state;      % desried state [x; y; z; xdot; ydot; zdot];
        rot;            % rotation matrix body to world
        vertices_b;     % position of vertices of the polygon payload in the body frame
        vertices_w;     % position of vertices of the polygon payload in the world frame

        color;          % color of Load
        alpha;          % alpha of the surface of Load

        state_hist        % position history
        state_des_hist;   % desired position history
        
        time_hist;      % time history
        max_iter;       % max iteration
    end

    properties (SetAccess = private)
        h_3d
        h_pos_hist;     % position history handle
        h_pos_des_hist; % desired position history handle
        h_vert_pos_hist   % polygon vertices postion history handle
        h_edge_hist     %polygon edges history handle
        text_dist;  % distance of quad number to quad
    end

    methods
        % Constructor
        function Q = LoadPlot(state, vertices, color, alpha, max_iter, h_3d)
            Q.state = state;
            Q.color = color;
            Q.alpha = alpha;
            Q.rot = QuatToRot(Q.state(7:10));
            Q.vertices_b = vertices;
            Q.vertices_w = rigidbody_polygon_pos(Q.state(1:3), Q.rot, Q.vertices_b);
            Q.text_dist = 0.01;
            Q.des_state = Q.state(1:6);
            Q.max_iter = max_iter;
            Q.state_hist = zeros(6, max_iter);
            Q.state_des_hist = zeros(6, max_iter);
            Q.time_hist = zeros(1, max_iter);

            % Initialize plot handle
            if nargin < 7, h_3d = gca; end
            Q.h_3d = h_3d;
            hold(Q.h_3d, 'on')
            Q.h_pos_hist = plot3(Q.h_3d, Q.state(1), Q.state(2), Q.state(3), 'r.');
            Q.h_pos_des_hist = plot3(Q.h_3d, Q.des_state(1), Q.des_state(2), Q.des_state(3), 'b.');
            Q.h_vert_pos_hist = drawPolyFromVertices(Q.vertices_w',Q.color,Q.alpha);
            
            hold(Q.h_3d, 'off')
        end

        % Update load state
        function UpdateLoadState(Q, state, time)
            Q.state = state;
            Q.time = time;
            Q.rot = QuatToRot(state(7:10))'; % Q.rot needs to be body-to-world
        end

        % Update desired load state
        function UpdateDesiredLoadState(Q, des_state)
            Q.des_state = des_state;
        end

        % Update load history
        function UpdateLoadHist(Q)
            Q.k = Q.k + 1;
            Q.time_hist(Q.k) = Q.time;
            Q.state_hist(:,Q.k) = Q.state(1:6);
            Q.state_des_hist(:,Q.k) = Q.des_state(1:6);
        end

        % Update vertices position
        function UpdateVerticesPos(Q)
            Q.vertices_w = rigidbody_polygon_pos(Q.state(1:3), Q.rot, Q.vertices_b);
        end

        % Truncate history
        function TruncateHist(Q)
            Q.time_hist = Q.time_hist(1:Q.k);
            Q.state_hist = Q.state_hist(:, 1:Q.k);
            Q.state_des_hist = Q.state_des_hist(:, 1:Q.k);
        end

        % Update load plot
        function UpdateLoadPlot(Q, state, des_state, time)
            Q.UpdateLoadState(state, time);
            Q.UpdateDesiredLoadState(des_state);
            Q.UpdateLoadHist();
            Q.UpdateVerticesPos();
            [Vtx_X, Vtx_Y, Vtx_Z] = verticesToconvhull(Q.vertices_w');
            set(Q.h_vert_pos_hist, ...
                'XData', Vtx_X, ...
                'YData', Vtx_Y, ...
                'ZData', Vtx_Z);
            set(Q.h_pos_hist, ...
                'XData', Q.state_hist(1,1:Q.k), ...
                'YData', Q.state_hist(2,1:Q.k), ...
                'ZData', Q.state_hist(3,1:Q.k));
            set(Q.h_pos_des_hist, ...
                'XData', Q.state_des_hist(1,1:Q.k), ...
                'YData', Q.state_des_hist(2,1:Q.k), ...
                'ZData', Q.state_des_hist(3,1:Q.k));
%             set(Q.h_edge_hist, ...
%                 'XData', Vtx_X, ...
%                 'YData', Vtx_Y, ...
%                 'ZData', Vtx_Z);
            drawnow;
        end
    end

end
