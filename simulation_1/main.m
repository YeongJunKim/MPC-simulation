
addpath('../utils/')



clear all
clf


global app
MODE_GO_FOLLOW = 1;
MODE_STOP_CONTAINMENT = 2;
MODE_TARGET_SENSING = 3;
%% init
app.agent_num = 8;
app.leader_num = 4;
app.follower_num = app.agent_num - app.leader_num;

app.initial_states = zeros(3, app.agent_num);
app.initial_states(:,1) = [0.0 0.0 0]';
app.initial_states(:,2) = [1.0 0.0 0]';
app.initial_states(:,3) = [0.5 1.0 0]';
app.initial_states(:,4) = [0.0 1.0 0]';
app.initial_states(:,5) = [0.0 1.5 0]';
app.initial_states(:,6) = [0.5 -1.5 0]';
app.initial_states(:,7) = [2 2 0]';
app.initial_states(:,8) = [5 1 0]';

app.states = zeros(2, app.agent_num);
app.states(:,1) = [0.0 0.0]';
app.states(:,2) = [1.0 0.0]';
app.states(:,3) = [0.5 1.0]';
app.states(:,4) = [0.0 1.0]';
app.states(:,5) = [0.0 1.5]';
app.states(:,6) = [0.5 -1.5]';
app.states(:,7) = [2 2]';
app.states(:,8) = [5 1]';

app.states_ref = zeros(3, app.agent_num);
app.states_ref(:,1) = [2.0 2.0 0]';
app.states_ref(:,2) = [5.0 2.0 0]';
app.states_ref(:,3) = [5.0 5.0 0]';
app.states_ref(:,4) = [2.5 5.0 0]';
app.states_ref(:,5) = [0.0 1.5 0]';
app.states_ref(:,6) = [0.5 -1.5 0]';
app.states_ref(:,7) = [2 2 0]';
app.states_ref(:,8) = [5 1 0]';

app.control_input = zeros(2, app.agent_num);

app.dt = 0.05;                   % sampling time (unit: second)

app.MODE = 0;
%% Plotting inintialization
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


%% app.mpc settting

mpc_initialization();
app.history_size = 50;
hbar = waitbar(0, 'Simulation Progress');
for k=1:app.history_size
    for ct = 1:app.agent_num
        yk = app.mpc.agent(ct).data.xHistory(1:3,k)' + randn + 0.01;
        xk = correct(app.mpc.agent(ct).data.EKF, yk);
        [uk, options] = nlmpcmove(app.mpc.agent(ct).data.nlobj, xk, app.mpc.agent(ct).data.lastMV,app.mpc.agent(ct).data.ref',[],app.mpc.agent(ct).data.options);
        predict(app.mpc.agent(ct).data.EKF,uk,0.4);
        lastMV = uk;
        ODEFUN = @(t,xk) FlyingRobotStateFcn(xk,uk);
        [TOUT,YOUT] = ode45(ODEFUN,[0 0.4], app.mpc.agent(ct).data.xHistory(:,k)');
        app.mpc.agent(ct).data.xHistory(:,k+1) = YOUT(end,:);
        app.mpc.agent(ct).data.uHistory(:,k+1) = uk;
    end
    waitbar(k/app.history_size, hbar);
end
close(hbar)

%% plotting history

figure(1);
clf;
legends = cell(1,app.agent_num);
for ct = 1:app.agent_num
    legends{ct} = num2str(ct);
    legends{ct} = strcat("robot",legends{ct});
    x = zeros(1, app.history_size+1);
    y = zeros(1, app.history_size+1);
    x(:) = app.mpc.agent(ct).data.xHistory(1,:);
    y(:) = app.mpc.agent(ct).data.xHistory(2,:);
    plot(x,y, '-+'); hold on;
end
legend(legends);
for ct = 1:app.agent_num
    FlyingRobotPlotTracking(app.mpc.agent(ct).data.xHistory, app.mpc.agent(ct).data.uHistory, ct);
end


%% Simulation
i = 0;
while(1)
    i = i + 1;
    MODE = 0;
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
        MODE = MODE_STOP_CONTAINMENT;
    end
    
    
    
    fprintf("MODE is %d  1 = STOP_CONTAINMENT , 2 = GO_FOLLOW , 3 = TARGET_SENSING \n", MODE);
    
    % MODE 1 : STOP_CONTAINMENT
    if(MODE == MODE_STOP_CONTAINMENT)
        % 
        STOP_CONTAINMENT(1);
    end 
    % MODE 2 : GO_FOLLOW
    if(MODE == MODE_GO_FOLLOW)
        %
        GO_FOLLOW(1);
    end
    % MODE 3 : TARGET_SENSING
    if(MODE == MODE_TARGET_SENSING)
        %
        TARGET_SENSING(1);
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








function r = dynamics(x, u)
r = zeros(3,1);
r(1) = x(1) + u(1) * cos(x(3)) * 0.1;
r(2) = x(2) + u(1) * sin(x(3)) * 0.1;
r(3) = x(3) + u(2) * 0.1;
end






