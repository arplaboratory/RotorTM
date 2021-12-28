function python_QuadPlot(max_iter,cstep,state_hist,state_des_hist,time_hist,map,path)
%PLOT_PYTHON_TRAJ Summary of this function goes here
%   Detailed explanation goes here
h_3d = gca;
qn = 1;
quadcolors = lines(qn);
basicdata  = map.basicdata;
[rowbasicdata ~] = size(basicdata);
if rowbasicdata >= 2
    block = basicdata(2:rowbasicdata,:);
else
    block = [];
end
finalpath = simplify_path(path{1},block,map);
plot_path(map, finalpath);
pause(20);
hold on
for iter = 1:max_iter
    tic
    time = time_hist(iter);
    x = state_hist(iter,:)';
    desired_state = state_des_hist(:,iter);
    if iter == 1
        QP = QuadPlot(qn, x, 0.1, 0.04, quadcolors(qn,:), max_iter, h_3d);
        h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time));
    end
    % Update quad plot
    QP.UpdateQuadPlot(x, desired_state, time);
    set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time))
    t = toc;
    %fprintf('the time is %d \n',t)
    
    % Pause to make real-time
    if (t < cstep)
        pause(cstep - t);
    end
    
end
end

