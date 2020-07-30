% paper: A hybrid model predictive control scheme for containment and distributed sensing in multi-agent systems
% author: Yeong Jun Kim(colson)
% email: colson@korea.ac.kr || dud3722000@naver.com
% date: 2020-07-28
% Requirement(In same workspace) :
% matlab_utils repository in https://www.github.com/YeongJunKim/matlab_utils

%%
addpath('./../../matlab_utils/')
addpath('./mode/')
clear all
clf
clc

% Generating Video flag
MAKE_VIDEO = 0;

global app
MODE_GO_FOLLOW          = 1;
MODE_STOP_CONTAINMENT   = 2;
MODE_TARGET_SENSING     = 3;
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
app.states_ref(:,4) = [0 7.0 pi/3 0 0 0]';
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

app.states_ref_arr = zeros(6, app.agent_num, 3);
app.states_ref_arr(:,:,1) = app.states_ref(:,:);

app.states_ref_arr(1:2,1,2) = [-3 -3];
app.states_ref_arr(1:2,2,2) = [2 -3];
app.states_ref_arr(1:2,3,2) = [2 2];
app.states_ref_arr(1:2,4,2) = [-3 9];
p = zeros(app.leader_num,2);
for j = 1:app.leader_num
    p(j,:) =app.states_ref_arr(1:2,j,2);
end
for i = 1:app.follower_num
    app.states_ref_arr(1:2,app.leader_num+i,2) = inhull_random_point(p,1);
end

app.states_ref_arr(1:2,1,3) = [-10 -10];
app.states_ref_arr(1:2,2,3) = [3 -10];
app.states_ref_arr(1:2,3,3) = [3 3];
app.states_ref_arr(1:2,4,3) = [-10 3];
p = zeros(app.leader_num,2);
for j = 1:app.leader_num
    p(j,:) =app.states_ref_arr(1:2,j,3);
end
for i = 1:app.follower_num
    app.states_ref_arr(1:2,app.leader_num+i,3) = inhull_random_point(p,1);
end
app.ref_change_flag = 1;

app.states_target = zeros(6, app.agent_num);
app.control_input = zeros(2, app.agent_num);


% app.states_ref_arr(:,:,1) = app.states_ref(:,:);
% app.states_ref_arr(:,:,2) = app.states_ref(:,:) + 1;
% app.states_ref_arr(:,:,3) = app.states_ref(:,:) + 1;


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
    if i <= app.leader_num
        app.plot_p{i} = plot(ax, app.states(1,i), app.states(2,i), 'ro'); hold on; grid on;
    else
        app.plot_p{i} = plot(ax, app.states(1,i), app.states(2,i), 'bo'); hold on; grid on;
    end
end
for i = 1:app.leader_num
    app.plot_t{i} = plot(ax, app.states_ref(1,i), app.states_ref(2,i), 'r*');
end
for i = 1:app.follower_num
    app.plot_ft{i} = plot(ax, app.states_ref(1, app.leader_num+i), app.states_ref(2,app.leader_num+i), 'b*');
end

p = zeros(app.leader_num, 2);
for j = 1:app.leader_num
    p(j,:) = app.states(1:2,j);
end
[k, av] = convhull(p);
app.convex_hull = plot(ax, p(k,1),p(k,2));

xlim([-13.0 11.0]); ylim([-13.0 11.0]);
xlabel('X (m)'); ylabel('Y (m)');
legend([app.plot_p{1}  app.plot_p{app.leader_num + 1} app.plot_t{1} app.plot_ft{1}], {'Leaders', 'Follower', 'Leaders target', 'Followers target'});
title('Containment MPC');
%% run simulation
% hbar = waitbar(s,'Simulation Progress');
for k = 1:app.simulation_step+2
    % Check shape of leaders.
    % Check all follower are in convex hull.
    p = zeros(app.leader_num,2);
    for j = 1:app.leader_num
        p(j,:) =app.states(1:2,j);
    end
    kkk = convhulln(p);
    % convex hull check
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
    ch = 0; % chage flag
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
    % Assuming that the surrounding environment is sensed immediately when converging to the target area, the next D(next) is given immediately.
    
    if(ch == 1)
        fprintf("app.mode exchanged : ");
        if(app.mode == MODE_GO_FOLLOW)
            app.states_target = app.states_ref;
            fprintf("STOP to GO(CONTAINEMNT to FOLLOW)");
        elseif(app.mode == MODE_STOP_CONTAINMENT)
            app.states_target(1:6,1:app.leader_num) = app.states(1:6,1:app.leader_num);
            p = zeros(app.leader_num,2);
            for j = 1:app.leader_num
                p(j,:) =app.states(1:2,j);
            end
            for i = 1:app.follower_num
                app.states_target(1:2,app.leader_num+i) = inhull_random_point(p,1);
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
             app.states_target(1:2,app.leader_num+i) = inhull_random_point(p,1);
        end
    elseif(app.mode == MODE_GO_FOLLOW)
        p = zeros(app.leader_num,2);
        for j = 1:app.leader_num
            p(j,:) = app.states_ref(1:2,j);
        end
        for j = 1:app.follower_num
%             app.states_target(1:2,app.leader_num+j) = inhull_random_point(p,1);
        end
        
    end
    % display mode in plot
    if(app.mode == MODE_GO_FOLLOW)
        ti = "Containment MPC";
        ti = strcat(ti, ",,MODE: GO FOLLOW");
        title(ti);
    elseif(app.mode == MODE_STOP_CONTAINMENT)
        ti = "Containment MPC";
        ti = strcat(ti, ",,MODE: STOP CONTAINMENT");
        title(ti);
    elseif(app.mode == MODE_TARGET_SENSING)
        ti = "Containment MPC";
        ti = strcat(ti, ",,MODE: TARGET SENSING");
        title(ti);
    end
    % control & estimate position with EKF
    
    
    if(app.mode == MODE_GO_FOLLOW)
       % ...(4)
       max = 0;
       min(1,1) = 10000;
       for ag = 1:app.agent_num
           diff = norm(abs(app.states(1:2,ag) - app.states_target(1:2,ag)))/app.mpc.agent(ct).data.input_max;
%              diff = norm(abs(app.states(1:2,ag) - app.states_target(1:2,ag)))/app.mpc.agent(ct).data.input_max; 
             if diff >= max
                 max = diff;
             end
             if diff <= min
                min = diff; 
             end
       end
       for ag = 1:app.leader_num
%         app.mpc.agent(ag).data.nlobj_tracking.ControlHorizon = round(max);
       end
    else
        for ag = 1:app.leader_num
%            app.mpc.agent(ag).data.nlobj_tracking.ControlHorizon = app.control_h; 
        end
    end
    
    
    for ct = 1:app.agent_num
        yk = app.mpc.agent(ct).data.xHistory(k,1:3)' + randn*0.01;
        xk = correct(app.mpc.agent(ct).data.EKF, yk);
        %         [uk, app.mpc.agent(ct).data.options] = nlmpcmove(app.mpc.agent(ct).data.nlobj_tracking, xk, app.mpc.agent(ct).data.lastMV,app.mpc.agent(ct).data.Xref(k:min(k+9,app.simulation_step+2),:),[],app.mpc.agent(ct).data.options);
       
        [uk, app.mpc.agent(ct).data.options] = nlmpcmove(app.mpc.agent(ct).data.nlobj_tracking, xk, app.mpc.agent(ct).data.lastMV,app.states_target(:,ct)',[],app.mpc.agent(ct).data.options);
        
        predict(app.mpc.agent(ct).data.EKF ,uk ,app.mpc.agent(ct).data.Ts);
        app.mpc.agent(ct).data.uHistory(k,:) = uk';
        app.mpc.agent(ct).data.lastMV = uk;
        ODEFUN = @(t,xk) FlyingRobotStateFcn(xk,uk);
        [TOUT,YOUT] = ode45(ODEFUN,[0 app.mpc.agent(ct).data.Ts], app.mpc.agent(ct).data.xHistory(k,:)');
        if(app.mode == MODE_STOP_CONTAINMENT)
           if ct <= app.leader_num && k > 1
              app.mpc.agent(ct).data.xHistory(k+1,:) = app.mpc.agent(ct).data.xHistory(k,:); 
           else
              app.mpc.agent(ct).data.xHistory(k+1,:) = YOUT(end,:);
           end
        else
            
              app.mpc.agent(ct).data.xHistory(k+1,:) = YOUT(end,:);
        end
        app.states(:,ct) = YOUT(end,:);
    end
    
    
    plot_update();
    % check converge
    % If all leaders are converge target positions,
    % Then D(next) is given.
    di = app.states_ref - app.states;
    ss = zeros(size(di,1),1);
    for ct = 1:app.agent_num
        ss = ss + abs(di(:,i));
    end
    ss = sum(ss);
    
    if(ss < 30)
        app.ref_change_flag = app.ref_change_flag + 1;
        if(app.ref_change_flag > size(app.states_ref_arr,3))
            fprintf("End trajectory.");
            break;
        end
        app.states_ref = app.states_ref_arr(:,:,app.ref_change_flag);
        fprintf("app.mode exchanged : SENSING\n");
        for i = 1:app.leader_num
            app.plot_t{i}.XData = app.states_ref(1,i);
            app.plot_t{i}.YData = app.states_ref(2,i);
        end
        fprintf("app.mode exchanged : SENSING END\n");
        app.mode = MODE_TARGET_SENSING;
    end
%     waitbar(k/(app.simulation_step+2), hbar);
    if(MAKE_VIDEO == 1)
        F(k) = getframe(gcf);
    end
end
% close(hbar)
%% plot history
for ct = 1:app.agent_num
    hold on;
    FlyingRobotPlotTracking_noinfo( app.mpc.agent(ct).data.Ts, app.mpc.agent(ct).data.p,(app.simulation_step+2),app.mpc.agent(ct).data.xHistory,app.mpc.agent(ct).data.uHistory,ct); drawnow;
end
%% result
if (MAKE_VIDEO == 1)
    video_name = sprintf('Containment MPC simulation_%s_%s',datestr(now,'yymmdd'),datestr(now,'HHMMSS'));
    video = VideoWriter(video_name,'MPEG-4');
    video.Quality = 100;
    video.FrameRate = 1/0.1;   % 영상의 FPS, 값이 클수록 영상이 빨라짐
    open(video);
    writeVideo(video,F);
    close(video);
end

%% plot update
function plot_update()
global app
% Update plot
    for j = 1:app.agent_num
        app.plot_p{j}.XData = app.states(1,j);
        app.plot_p{j}.YData = app.states(2,j);
    end
    for j = 1:app.follower_num
        app.plot_ft{j}.XData = app.states_target(1,app.leader_num+j);
        app.plot_ft{j}.YData = app.states_target(2,app.leader_num+j);
    end
    % draw convel hull
    p = zeros(app.leader_num,2);
    for j = 1:app.leader_num
        p(j,:) =app.states(1:2,j);
    end
    [kk, av] = convhull(p);
    app.convex_hull.XData = p(kk,1); app.convex_hull.YData = p(kk,2);
    drawnow;
end

