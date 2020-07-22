%% Flying Robot 
% * x(1) - x inertial coordinate of center of mass
% * x(2) - y inertial coordinate of center of mass
% * x(3) - theta, robot (thrust) direction
% * x(4) - vx, velocity of x
% * x(5) - vy, velocity of y
% * x(6) - omega, angular velocity of theta
% For more information on the flying robot, see [1]. The model in the paper
% uses two thrusts ranging from -1 to 1. However, this example assumes that
% there are four physical thrusts in the robot, ranging from 0 to 1, to
% achieve the same control freedom.

%% Trajectory Planning
% The robot initially rests at |[-10,-10]| with an orientation angle of
% |pi/2| radians (facing north). The flying maneuver for this example is to
% move and park the robot at the final location |[0,0]| with an angle of
% |0| radians (facing east) in |12| seconds. The goal is to find the
% optimal path such that the total amount of fuel consumed by the thrusters
% during the maneuver is minimized.
%
% Nonlinear MPC is an ideal tool for trajectory planning problems because
% it solves an open-loop constrained nonlinear optimization problem given
% the current plant states. With the availability of a nonlinear dynamic
% model, MPC can make more accurate decisions.
%
% Create a nonlinear MPC object with |6| states, |6| outputs, and |4|
% inputs. By default, all the inputs are manipulated variables (MVs).
nx = 6;
ny = 6;
nu = 4;
nlobj = nlmpc(nx,ny,nu);

%%
% Specify the prediction model state function using the function name. You
% can also specify functions using a function handle. For details on the
% state function, open |FlyingRobotStateFcn.m|. For more information on
% specifying the prediction model, see
% <docid:mpc_ug#mw_cda3c7d0-97e5-4c33-9ec5-dcf440b4c5cf>.
nlobj.Model.StateFcn = "FlyingRobotStateFcn";

%%
% Specify the Jacobian of the state function using a function handle. It is
% best practice to provide an analytical Jacobian for the prediction model.
% Doing so significantly improves simulation efficiency. For details on the
% Jacobian function, open |FlyingRobotStateJacobianFcn.m|.
nlobj.Jacobian.StateFcn = @FlyingRobotStateJacobianFcn;

%%
% For this example, the target prediction time is |12| seconds. Therefore,
% specify a sample time of |0.4| seconds and prediction horizon of |30|
% steps.
Ts = 0.1;
p = 5;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;

%%
% To perform trajectory planning instead of feedback control, use the
% maximum control horizon, which provides the maximum number of decision
% variables for the optimization problem. Since trajectory planning usually
% runs at a much slower sampling rate than a feedback controller, the extra
% computation load introduced by a large control horizon can be tolerated.
% Set the control horizon equal to the prediction horizon.
nlobj.ControlHorizon = p;

%%
% A trajectory planning problem usually involves a nonlinear cost function,
% which can be used to find the shortest distance, the maximal profit, or
% as in this case, the minimal fuel consumption. Because the thrust value
% is a direct indicator of fuel consumption, compute the fuel cost as the
% sum of the thrust values used across the prediction horizon. Specify this
% cost function using an anonymous function handle. For more information on
% specifying cost functions, see
% <docid:mpc_ug#mw_c432ddbe-6685-4cf2-be0e-2d46e07fed51>.
nlobj.Optimization.CustomCostFcn = @(X,U,e,data) Ts*sum(sum(U(1:p,:)));

%%
% For this example, the custom cost function replaces the default cost
% function that is typically used in feedback control.
nlobj.Optimization.ReplaceStandardCost = true;

%%
% The goal of the maneuver is to park the robot at |[0,0]| with an angle of
% |0| radians at the 12th second. Specify this goal as equality constraints
% on the states, where every position and velocity state at the last
% prediction step should be zero. For more information on specifying
% constraint functions, see
% <docid:mpc_ug#mw_80f1880d-e73b-401f-a23d-0084948d9bc1>.
nlobj.Optimization.CustomEqConFcn = @(X,U,data) X(end,:)';

%%
% It is best practice to provide analytical Jacobian functions for your
% custom cost and constraint functions as well. However, this example
% intentionally skips them so that their Jacobian is computed by the
% nonlinear MPC controller using the built-in numerical perturbation
% method.

%%
% Each thrust has an operating range between |0| and |1|, which is
% translated into lower and upper bounds on the MVs.
for ct = 1:nu
    nlobj.MV(ct).Min = 0;
    nlobj.MV(ct).Max = 0.5;
end

%%
% Specify the initial conditions for the robot.
x0 = [-10 -10 pi/2 0 0 0]';  % robot parks at [-10, -10], facing north
u0 = zeros(nu,1);           % thrust is zero

%%
% It is best practice to validate the user-provided model, cost, and
% constraint functions and their Jacobians. To do so, use the
% |validateFcns| command.
fprintf("validate\n");
validateFcns(nlobj,x0,u0);
fprintf("validate end\n");
%%
% The optimal state and MV trajectories can be found by calling the
% |nlmpcmove| command once, given the current state |x0| and last MV |u0|.
% The optimal cost and trajectories are returned as part of the |info|
% output argument.
[~,~,info] = nlmpcmove(nlobj,x0,u0);

%%
% Plot the optimal trajectory. The optimal cost is 7.8.
FlyingRobotPlotPlanning(info);

%%
% The first plot shows the optimal trajectory of the six robot states
% during the maneuver. The second plot shows the corresponding optimal MV
% profiles for the four thrusts. The third plot shows the X-Y position
% trajectory of the robot, moving from |[-10 -10 pi/2]| to |[0 0 0]|.

%% Feedback Control for Path Following
% After the optimal trajectory is found, a feedback controller is required
% to move the robot along the path. In theory, you can apply the optimal MV
% profile directly to the thrusters to implement feed-forward control.
% However, in practice, a feedback controller is needed to reject
% disturbances and compensate for modeling errors.
%
% You can use different feedback control techniques for tracking. In this
% example, you use another nonlinear MPC controller to move the robot to
% the final location. In this path tracking problem, you track references
% for all six states.
nlobj_tracking = nlmpc(nx,ny,nu);

%% 
% Use the same state function and its Jacobian function.
nlobj_tracking.Model.StateFcn = nlobj.Model.StateFcn;
nlobj_tracking.Jacobian.StateFcn = nlobj.Jacobian.StateFcn;

%%
% For feedback control applications, reduce the computational effort by
% specifying shorter prediction and control horizons.
nlobj_tracking.Ts = Ts;
nlobj_tracking.PredictionHorizon = 10;
nlobj_tracking.ControlHorizon = 4;

%% 
% The default cost function in nonlinear MPC is a standard quadratic cost
% function suitable for reference tracking and disturbance rejection. For
% tracking, the states have higher priority (larger penalty weights) than
% the MV moves.
nlobj_tracking.Weights.ManipulatedVariablesRate = 0.2*ones(1,nu);
nlobj_tracking.Weights.OutputVariables = 5*ones(1,nx);

%% 
% Set the same bounds for the thruster inputs.
for ct = 1:nu
    nlobj_tracking.MV(ct).Min = 0;
    nlobj_tracking.MV(ct).Max = 1;
end

%%
% Also, to reduce fuel consumption, it is clear that |u1| and |u2| cannot be
% positive at any time during the operation. Therefore, implement equality
% constraints such that |u(1)*u(2)| must be |0| for all prediction steps.
% Apply similar constraints for |u3| and |u4|.
nlobj_tracking.Optimization.CustomEqConFcn = ...
    @(X,U,data) [U(1:end-1,1).*U(1:end-1,2); U(1:end-1,3).*U(1:end-1,4)];

%%
% Validate your prediction model and custom functions, and their Jacobians.
validateFcns(nlobj_tracking,x0,u0);

%% Nonlinear State Estimation
% In this example, only the three position states (x, y and angle) are
% measured. The velocity states are unmeasured and must be estimated. Use
% an extended Kalman filter (EKF) from Control System Toolbox(TM) for
% nonlinear state estimation.
%
% Because an EKF requires a discrete-time model, you use the trapezoidal
% rule to transition from x(k) to x(k+1), which requires the solution of
% |nx| nonlinear algebraic equations. For more information, open
% |FlyingRobotStateFcnDiscreteTime.m|.
DStateFcn = @(xk,uk,Ts) FlyingRobotStateFcnDiscreteTime(xk,uk,Ts);

%%
% Measurement can help the EKF correct its state estimation. Only the first
% three states are measured.
DMeasFcn = @(xk) xk(1:3);

%%
% Create the EKF, and indicate that the measurements have little noise.
EKF = extendedKalmanFilter(DStateFcn,DMeasFcn,x0);
EKF.MeasurementNoise = 0.01;

%% Closed-Loop Simulation of Tracking Control
% Simulate the system for |32| steps with correct initial conditions.
Tsteps = 32;        
xHistory = x0';
uHistory = [];
lastMV = zeros(nu,1);

%%
% The reference signals are the optimal state trajectories computed at the
% planning stage. When passing these trajectories to the nonlinear MPC
% controller, the current and future trajectory is available for
% previewing.
Xopt = info.Xopt;
Xref = [Xopt(2:p+1,:);repmat(Xopt(end,:),Tsteps-p,1)];

%% Use |nlmpcmove| and |nlmpcmoveopt| command for closed-loop simulation.
hbar = waitbar(0,'Simulation Progress');
options = nlmpcmoveopt;
for k = 1:Tsteps
    % Obtain plant output measurements with sensor noise.
    yk = xHistory(k,1:3)' + randn*0.01;
    % Correct state estimation based on the measurements.
    xk = correct(EKF, yk);
    % Compute the control moves with reference previewing.
    [uk,options] = nlmpcmove(nlobj_tracking,xk,lastMV,Xref(k:min(k+9,Tsteps),:),[],options);
    disp("uk");
    disp(uk);
    % Predict the state for the next step.
    predict(EKF,uk,Ts);
    % Store the control move and update the last MV for the next step.
    uHistory(k,:) = uk';
    lastMV = uk;
    % Update the real plant states for the next step by solving the
    % continuous-time ODEs based on current states xk and input uk.
    ODEFUN = @(t,xk) FlyingRobotStateFcn(xk,uk);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
    % Store the state values.
    xHistory(k+1,:) = YOUT(end,:);            
    % Update the status bar.
    waitbar(k/Tsteps, hbar);
end
close(hbar)

FlyingRobotPlotTracking(info,Ts,p,Tsteps,xHistory,uHistory);


