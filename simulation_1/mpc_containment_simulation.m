
addpath('../utils/')
addpath('./mode/')
clear all
% close all
clf
clc


global app
MODE_GO_FOLLOW = 1;
MODE_STOP_CONTAINMENT = 2;
MODE_TARGET_SENSING = 3;
%% init
app.mode = MODE_TARGET_SENSING;
app.dt = 0.3;
app.agent_num = 8;
app.leader_num = 4;
app.follower_num = app.agent_num - app.leader_num;

app.initial_states = zeros(6, app.agent_num);
app.initial_states(:,1) = [-10.0 -10.0 pi/2 0 0 0]';
app.initial_states(:,2) = [-6 -11 pi/2 0 0 0]';
app.initial_states(:,3) = [-3 -7.0 pi/2 0 0 0]';
app.initial_states(:,4) = [-6 -1.0 pi/2 0 0 0]';
app.initial_states(:,5) = [-5 -1.5 pi/2 0 0 0]';
app.initial_states(:,6) = [-3 -1.5 pi/2 0 0 0]';
app.initial_states(:,7) = [-9  -3 pi/2 0 0 0]';
app.initial_states(:,8) = [-2 -8 pi/2 0 0 0]';

app.states_ref = zeros(6, app.agent_num);
app.states_ref(:,1) = [0.0 0.0 pi 0 0 0]';
app.states_ref(:,2) = [5.0 0.0 pi/2 0 0 0]';
app.states_ref(:,3) = [5.0 5.0 0 0 0 0]';
app.states_ref(:,4) = [2.5 5.0 pi/3 0 0 0]';
app.states_ref(:,5) = [2 1.5 0 0 0 0]';
app.states_ref(:,6) = [0.5 -1.5 0 0 0 0]';
app.states_ref(:,7) = [-1 5 0 0 0 0]';
app.states_ref(:,8) = [3 6 0 0 0 0]';
p = zeros(app.leader_num,2);
    for j = 1:app.leader_num
        p(j,:) =app.states_ref(1:2,j);
    end
for i = 1:app.follower_num
    app.states_ref(1:2,app.leader_num+i) = inhull_random_point(p,1);
end


app.states_target = zeros(6, app.agent_num);
app.states_temp_ref = zeros(6, app.agent_num);
app.states_temp_ref = app.states_ref;

app.control_input = zeros(2, app.agent_num);




%% app.mpc settting
app.simulation_step = 100;

mpc_containment_simulation_init(0);

%% plotting initialization
figure(1);
clf;
ax = axes;

app.states = zeros(6,app.agent_num,app.simulation_step);
app.states(:,:,1) = app.initial_states(:,:);

app.plot_p = cell(1, app.agent_num);
app.plot_t = cell(1, app.leader_num);
app.plot_ft = cell(1, app.follower_num);


for i = 1:app.agent_num
    app.plot_p{i} = plot(ax, app.states(1,i), app.states(2,i), 'ro'); hold on; grid on;
end
for i = 1:app.leader_num
    app.plot_t{i} = plot(ax, app.states_ref(1,i), app.states_ref(2,i), 'r*');
end
for i = 1:app.follower_num
    app.plot_ft{i} = plot(ax, app.states_ref(1, app.leader_num+i), app.states_ref(2,app.leader_num+i), 'b+');
end

p = zeros(app.leader_num, 2);
for j = 1:app.leader_num
    p(j,:) = app.states(1:2,j);
end
[k, av] = convhull(p);
convex_hull = plot(ax, p(k,1),p(k,2));

xlim([-11.0 7.0]); ylim([-11.0 7.0]);
xlabel('X (m)'); ylabel('Y (m)');
legend([app.plot_p{1}  app.plot_p{4} app.plot_t{1}], {'Leaders', 'Follower', 'Target'});
title('Containment MPC');

%% RUN
hbar = waitbar(0,'Simulation Progress');
for k = 1:app.simulation_step+2
    % Check shape of leaders.
    
    % Check all follower are in convex hull.
    p = zeros(app.leader_num,2);
    for j = 1:app.leader_num
        p(j,:) =app.states(1:2,j);
    end
    [kk, av] = convhull(p);
    kkk = convhulln(p);
    check = 1;
    for j = 1:app.follower_num
        in = inhull(app.states(1:2,app.leader_num+j)',p,kkk);
        if(in == 0)
            fprintf("follower %d is not in convex hull. \n", j);
            check = 0;
        end
    end
    if(in ~= 0)
        fprintf("All followers arein convex hull. \n");
    end
    ch = 0;
    if(check == 0)
        if(app.mode == MODE_GO_FOLLOW)
            app.mode = MODE_STOP_CONTAINMENT;
            ch = 1;
        elseif(app.mode == MODE_STOP_CONTAINMENT)
            app.mode = MODE_STOP_CONTAINMENT;
        elseif(app.mode == MODE_TARGET_SENSING)
            app.mode = MODE_STOP_CONTAINMENT;
            ch = 1;
        end
    else
       if(app.mode == MODE_GO_FOLLOW)
           app.mode = MODE_GO_FOLLOW;
       elseif(app.mode == MODE_STOP_CONTAINMENT)
           app.mode = MODE_GO_FOLLOW;
           ch = 1;
       elseif(app.mode == MODE_TARGET_SENSING)
           app.mode = MODE_GO_FOLLOW;
           ch = 1;
       end
    end
    % target sensing이 필요 없는게 안정권에 들어왔을때 팔로워가 주변 환경을 센싱하게 되는데 센싱된다는 가정 하예
    % 다음 D(next)를 바로 줘버리면 끝나는거임, 그러니까 target polytope에 어느정도 수렴하면 D(next)를
    % 주면됨!
    if(ch == 1)
        fprintf("app.mode exchanged : ");
        if(app.mode == MODE_GO_FOLLOW)
            app.states_ref = app.states_temp_ref;
            fprintf("STOP to GO(CONTAINEMNT to FOLLOW)");
        elseif(app.mode == MODE_STOP_CONTAINMENT)
            
            app.states_temp_ref = app.states_ref;
            
            app.states_ref(1:6,1:app.leader_num) = app.states(1:6,1:app.leader_num);
            
            p = zeros(app.leader_num,2);
            for j = 1:app.leader_num
                p(j,:) =app.states(1:2,j);
            end
            for i = 1:app.follower_num
                app.states_ref(1:2,app.leader_num+i) = inhull_random_point(p,1);
            end
            fprintf("GO to STOP(FOLLOW to CONTAINMENT)");
        end
        fprintf("\n");
    end
    
    if(app.mode == MODE_STOP_CONTAINMENT)
        p = zeros(app.leader_num,2);
        for j = 1:app.leader_num
            p(j,:) =app.states(1:2,j);
        end
        for i = 1:app.follower_num
            app.states_ref(1:2,app.leader_num+i) = inhull_random_point(p,1);
        end
    end
    
    for ct = 1:app.agent_num
        yk = app.mpc.agent(ct).data.xHistory(k,1:3)' + randn*0.01;
        xk = correct(app.mpc.agent(ct).data.EKF, yk);
        %         [uk, app.mpc.agent(ct).data.options] = nlmpcmove(app.mpc.agent(ct).data.nlobj_tracking, xk, app.mpc.agent(ct).data.lastMV,app.mpc.agent(ct).data.Xref(k:min(k+9,app.simulation_step+2),:),[],app.mpc.agent(ct).data.options);
        [uk, app.mpc.agent(ct).data.options] = nlmpcmove(app.mpc.agent(ct).data.nlobj_tracking, xk, app.mpc.agent(ct).data.lastMV,app.states_ref(:,ct)',[],app.mpc.agent(ct).data.options);
        
        predict(app.mpc.agent(ct).data.EKF ,uk ,app.mpc.agent(ct).data.Ts);
        app.mpc.agent(ct).data.uHistory(k,:) = uk';
        app.mpc.agent(ct).data.lastMV = uk;
        ODEFUN = @(t,xk) FlyingRobotStateFcn(xk,uk);
        [TOUT,YOUT] = ode45(ODEFUN,[0 app.mpc.agent(ct).data.Ts], app.mpc.agent(ct).data.xHistory(k,:)');
        app.mpc.agent(ct).data.xHistory(k+1,:) = YOUT(end,:);
        app.states(:,ct) = YOUT(end,:);
    end
    for ct = 1:app.agent_num
        hold on;
%         FlyingRobotPlotTracking_noinfo( app.mpc.agent(ct).data.Ts, app.mpc.agent(ct).data.p,(app.simulation_step+2),app.mpc.agent(ct).data.xHistory,app.mpc.agent(ct).data.uHistory,ct); drawnow;
    end
    
    % Update plot
    for j = 1:app.agent_num
        app.plot_p{j}.XData = app.states(1,j);
        app.plot_p{j}.YData = app.states(2,j);
    end
    for j = 1:app.leader_num
        app.plot_t{j}.XData = app.states_ref(1,j); 
        app.plot_t{j}.YData = app.states_ref(2,j); 
    end
    for j = 1:app.follower_num
        app.plot_ft{j}.XData = app.states_ref(1,app.leader_num+j);
        app.plot_ft{j}.XData = app.states_ref(2,app.leader_num+j);
    end
    % draw convel hull
    convex_hull.XData = p(kk,1); convex_hull.YData = p(kk,2);
    drawnow;
    
    waitbar(k/(app.simulation_step+2), hbar);
end
close(hbar)
%% result



