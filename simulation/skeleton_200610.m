% Skeleton code for new MPC design
% Purpose: Achieve target position (or formation) while maintaining containment
% Based on this code, let's practice and develop our method.

%% Initialization
% Parameters
dt = 0.1;                   % sampling time (unit: second)

% State of robots
% Here, we consider position values as state, but it will be pose (position + heading angle) later.
% The number of leaders and followers can be increased later.
state_L1 = [0.0 0.0]';      % state of leader 1 (unit: meter)
state_L2 = [1.0 0.0]';      % state of leader 2
state_L3 = [0.5 1.0]';      % state of leader 3
state_F1 = [0.0 1.0]';      % state of follower 1, initially it is outside of the convex hull.

% Target State of robots
% Here, we consider that target positions for leader robots are known.
% If we assume that other values are given to leader robots (e.g. distance between leaders), the variables below would be changed.
state_ref_L1 = [2.0 2.0]';
state_ref_L2 = [3.0 2.0]';
state_ref_L3 = [2.5 3.0]';

% Measurements for robots
% Here, we consider x and y displacement between each leader and follower as measurements.
delta_F1_L1 = state_F1 - state_L1;
delta_F1_L2 = state_F1 - state_L2;
delta_F1_L3 = state_F1 - state_L3;

% Control input for robots
% Here, we consider x and y velocities as control input, but it can be linear and angular velocities later.
ctrl_L1 = [0.0 0.0]';       % control input for leader 1 (unit: meter/second)
ctrl_L2 = [0.0 0.0]';       % control input for leader 2
ctrl_L3 = [0.0 0.0]';       % control input for leader 3
ctrl_F1 = [0.0 0.0]';       % control input for follower 1

%% Plotting initialization
figure();
ax = axes;
robot_L1 = plot(ax, state_L1(1), state_L1(2), 'ro'); hold on; grid on;
robot_L2 = plot(ax, state_L2(1), state_L2(2), 'ro');
robot_L3 = plot(ax, state_L3(1), state_L3(2), 'ro');
robot_F1 = plot(ax, state_F1(1), state_F1(2), 'bo');
robot_target_L1 = plot(ax, state_ref_L1(1), state_ref_L1(2), 'r*');
robot_target_L2 = plot(ax, state_ref_L2(1), state_ref_L2(2), 'r*');
robot_target_L3 = plot(ax, state_ref_L3(1), state_ref_L3(2), 'r*');
xlim([-1.0 4.0]); ylim([-1.0 4.0]);
xlabel('X (m)'); ylabel('Y (m)');
legend([robot_L1 robot_F1 robot_target_L1], {'Leaders', 'Follower', 'Target'});
title('Containment MPC');

%% Simulation
num_iteration = 100;        % the number of iteration. You may change this value.
for i = 1:num_iteration
    % Measurements
    delta_F1_L1 = state_F1 - state_L1;
    delta_F1_L2 = state_F1 - state_L2;
    delta_F1_L3 = state_F1 - state_L3;
    
    % Design controller (ctrl_L1 ~ L3 and F1)
    % Just freely design your controller first.
    % We will decide a method to be implemented precisely later.
%     ctrl_L1 = 
%     ctrl_L2 = 
%     ctrl_L3 = 
%     ctrl_F1 = 
    
    % Update state
    state_L1 = update_state(state_L1, ctrl_L1, dt);
    state_L2 = update_state(state_L2, ctrl_L2, dt);
    state_L3 = update_state(state_L3, ctrl_L3, dt);
    state_F1 = update_state(state_F1, ctrl_F1, dt);
    
    % Update plot
    robot_L1.XData = state_L1(1); robot_L1.YData = state_L1(2);
    robot_L2.XData = state_L2(1); robot_L2.YData = state_L2(2);
    robot_L3.XData = state_L3(1); robot_L3.YData = state_L3(2);
    robot_F1.XData = state_F1(1); robot_F1.YData = state_F1(2);
    drawnow;
end

function output = update_state(state, ctrl, sampling_time)
    output = state + ctrl * sampling_time;
end