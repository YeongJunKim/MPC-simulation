% Skeleton code for new MPC design
% Purpose: Achieve target position (or formation) while maintaining containment
% Based on this code, let's practice and develop our method.
clear all
clf
%% Initialization
global app

app.agent_num = 10;
app.leader_num = 4;
app.follower_num = app.agent_num - app.leader_num;

app.states = zeros(2, app.agent_num);
app.states(:,1) = [0.0 0.0]';
app.states(:,2) = [1.0 0.0]';
app.states(:,3) = [0.5 1.0]';
app.states(:,4) = [0.0 1.0]';
app.states(:,5) = [0.0 1.5]';
app.states(:,6) = [0.5 -1.5]';
app.states(:,7) = [2 2]';
app.states(:,8) = [5 1]';
app.states(:,9) = [2.4 1]';
app.states(:,10) = [1 -3]';


app.states_ref = zeros(2, app.leader_num);
app.states_ref(:,1) = [2.0 2.0]';
app.states_ref(:,2) = [5.0 2.0]';
app.states_ref(:,3) = [5.0 5.0]';
app.states_ref(:,4) = [2.5 5.0]';

app.control_input = zeros(2, app.agent_num);

% Parameters
app.dt = 0.05;                   % sampling time (unit: second)


%% Plotting initialization
figure(1);
ax = axes;

app.plot_p = cell(1, app.agent_num);
app.plot_t = cell(1, app.leader_num);


for i = 1:app.agent_num
    app.plot_p{i} = plot(ax, app.states(1,i), app.states(2,i), 'ro'); hold on; grid on;
end
for i = 1:app.leader_num
    app.plot_t{i} = plot(ax, app.states_ref(1,i), app.states_ref(2,i), 'r*');
end

p = zeros(app.leader_num, 2);
for j = 1:app.leader_num
    p(j,:) = app.states(:,j);
end
[k, av] = convhull(p);
convex_hull = plot(ax, p(k,1),p(k,2));

xlim([-2.0 5.0]); ylim([-2.0 5.0]);
xlabel('X (m)'); ylabel('Y (m)');
legend([app.plot_p{1}  app.plot_p{4} app.plot_t{1}], {'Leaders', 'Follower', 'Target'});
title('Containment MPC');

%% Simulation
i = 0;
while(1)
    i = i + 1;
    % Check converge
    diff = zeros(2, app.leader_num);
    diff_norm = zeros(1, app.leader_num);
    for j = 1:app.leader_num
        diff(:, j) = app.states(:,j) - app.states_ref(:,j);
        diff_norm(j) = norm(diff(:,j));
    end
    if mean(diff_norm) < 0.05
        break;
    end
    
    % We need to check leader's shape is convex?
    
    
    % We need to check if all follower are in convex hull.
    p = zeros(app.leader_num, 2);
    for j = 1:app.leader_num
        p(j,:) = app.states(:,j);
    end
    
    [kk, av] = convhull(p);
    
    k = convhulln(p);
    check = 1;
    for j = 1:app.follower_num
        in = inhull(app.states(:,app.leader_num + j)', p, k);
        if(in == 0)
            fprintf("follower %d is not in convex hull. \n", j);
            check = 0;
        end
    end
    if(in ~= 0)
        fprintf("All followers are in convex hull. \n", j);
    end
    
    
    
    % control
    app.control_input(:,:) = 0;
    if(check == 1)
        % In convex
        app.control_input(:,:) = 0;
        s = size(kk,1) - 1;
        sum = 0;
        for j = 1:s
            sum = sum + app.states(:,kk(j));
        end
        sum = sum / s;
        for j = 1:app.leader_num
            app.control_input(:,j) = (app.states_ref(:,j) - app.states(:,j)) * 0.1;
        end
        
        for j = 1:app.follower_num
            diff = zeros(2,app.leader_num);
            distance = zeros(1,app.leader_num);
            for k = 1:app.leader_num
                diff(:,k) = app.states(:,k) - app.states(:,app.leader_num + j);
                distance(k) = norm(diff);
            end
            
            distance_n = normalize(distance, 'norm', 1);
            
            for k = 1:app.leader_num    
                app.control_input(:,app.leader_num + j) = app.control_input(:,app.leader_num + j) + diff(:,k) * distance_n(k)* 1.2;
                app.control_input(:,app.leader_num + j) = app.control_input(:,app.leader_num + j) - diff(:,k) * (1 -distance_n(k)) * 0.01;
            end
            
            diff = zeros(2, app.follower_num);
            distance = zeros(1, app.follower_num);
            for k = 1:app.follower_num
                diff(:,k) = app.states(:,app.leader_num + k) - app.states(:,app.leader_num + j);
                distance(k) = norm(diff(:,k));
            end
            
            distance_n = normalize(distance, 'norm' , 1);
            distance_n = 1 - distance_n;
            
            
            
            for k = 1:app.follower_num
                if k == j
                    continue;
                end
                app.control_input(:,app.leader_num + j) = app.control_input(:,app.leader_num + j) - diff(:,k)/norm(diff(:,k)) *  distance_n(k) * 0.1;
            end
        end
    else
        % Not in convex
        % Should do goal in the convex hull for every followers
        app.control_input(:,:) = 0;
        s = size(kk,1) - 1;
        sum = 0;
        for j = 1:s
            sum = sum + app.states(:,kk(j));
        end
        sum = sum / s;
        for j = 1:app.leader_num
            app.control_input(:,j) = (app.states_ref(:,j) - app.states(:,j)) * 0.01;
        end
        
        for j = 1:app.follower_num
            diff = zeros(2,app.leader_num);
            distance = zeros(1,app.leader_num);
            for k = 1:app.leader_num
                diff(:,k) = app.states(:,k) - app.states(:,app.leader_num + j);
                distance(k) = norm(diff(:,k));
            end
            
            distance_n = normalize(distance, 'norm', 1);
            
            for k = 1:app.leader_num
                app.control_input(:,app.leader_num + j) = app.control_input(:,app.leader_num + j) + diff(:,k) * distance_n(k) * 1.2;
                app.control_input(:,app.leader_num + j) = app.control_input(:,app.leader_num + j) - diff(:,k) * (1 -distance_n(k)) * 0.01;
            end
            
            diff = zeros(2, app.follower_num);
            distance = zeros(1, app.follower_num);
            for k = 1:app.follower_num
                diff(:,k) = app.states(:,app.leader_num + k) - app.states(:,app.leader_num + j);
                distance(k) = norm(diff);
            end
            
            distance_n = normalize(distance, 'norm' , 1);
            distance_n = 1 - distance_n;
            
            for k = 1:app.follower_num
                if k == j
                    continue;
                end
                app.control_input(:,app.leader_num + j) = app.control_input(:,app.leader_num + j) - diff(:,k)/norm(diff(:,k)) *  distance_n(k) * 0.1;
            end
            
        end
    end
    
    
    
    % Check hardware constraint
    for j = 1:app.agent_num
       if(app.control_input(1,j) > 0.15)
           app.control_input(1,j) = 0.15;
       elseif app.control_input(1,j) < -0.15
          app.control_input(1,j) = -0.15; 
       end
       if(app.control_input(2,j) > 0.15)
           app.control_input(2,j) = 0.15;
       elseif app.control_input(2,j) < -0.15
          app.control_input(2,j) = -0.15; 
       end
    end
    
    
    % Update state
    for j = 1:app.agent_num
        app.states(:,j) = update_state(app.states(:,j), app.control_input(:,j), app.dt);
    end
    
    
    
    % Update plot
    for j = 1:app.agent_num
        app.plot_p{j}.XData = app.states(1,j);
        app.plot_p{j}.YData = app.states(2,j);
    end
    % draw convel hull
    convex_hull.XData = p(kk,1); convex_hull.YData = p(kk,2);
    drawnow;
end

function output = update_state(state, ctrl, sampling_time)
output = state + ctrl * sampling_time;
end

