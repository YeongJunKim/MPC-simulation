
addpath('../utils/')
addpath('./mode/')
clear all
close all
clf
clc


global app
MODE_GO_FOLLOW = 1;
MODE_STOP_CONTAINMENT = 2;
MODE_TARGET_SENSING = 3;
%% init
app.dt = 0.5;  
app.agent_num = 8;
app.leader_num = 4;
app.follower_num = app.agent_num - app.leader_num;

app.initial_states = zeros(6, app.agent_num);
app.initial_states(:,1) = [-10.0 -10.0 pi/2 0 0 0]';
app.initial_states(:,2) = [-9 -9 pi/2 0 0 0]';
app.initial_states(:,3) = [-3 -7.0 pi/2 0 0 0]';
app.initial_states(:,4) = [-6 -1.0 pi/2 0 0 0]';
app.initial_states(:,5) = [-5 -1.5 pi/2 0 0 0]';
app.initial_states(:,6) = [-3 -1.5 pi/2 0 0 0]';
app.initial_states(:,7) = [-9  -3 pi/2 0 0 0]';
app.initial_states(:,8) = [-2 -8 pi/2 0 0 0]';

app.states_ref = zeros(6, app.agent_num);
app.states_ref(:,1) = [2.0 2.0 pi 0 0 0]';
app.states_ref(:,2) = [5.0 2.0 pi/2 0 0 0]';
app.states_ref(:,3) = [5.0 5.0 0 0 0 0]';
app.states_ref(:,4) = [2.5 5.0 pi/3 0 0 0]';
app.states_ref(:,5) = [2 1.5 0 0 0 0]';
app.states_ref(:,6) = [0.5 -1.5 0 0 0 0]';
app.states_ref(:,7) = [-1 5 0 0 0 0]';
app.states_ref(:,8) = [3 6 0 0 0 0]';

app.control_input = zeros(2, app.agent_num);


%% app.mpc settting


app.simulation_step = 30;
mpc_simulation_init(0);

%% RUN
hbar = waitbar(0,'Simulation Progress');
for k = 1:app.simulation_step+2
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
    end         
    waitbar(k/(app.simulation_step+2), hbar);
end
close(hbar)
%% result
for ct = 1:app.agent_num
    hold on;
   FlyingRobotPlotTracking_noinfo( app.mpc.agent(ct).data.Ts, app.mpc.agent(ct).data.p,(app.simulation_step+2),app.mpc.agent(ct).data.xHistory,app.mpc.agent(ct).data.uHistory,ct); drawnow; 
end


