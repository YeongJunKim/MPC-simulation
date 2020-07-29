function mpc_containment_simulation_init(generate_trajectory)
% argument
% generate_trajectory
% If setting 1. Then generate trajectory from app.mpc.agent(ct).data.x_ref.
% If setting 0. Dose not generate trajectory.
% If you want generate trajectory please consider hardware constraint.
% dt, p, horizon_size, ...

global app
app.ny = 6;
app.nx = 6;
app.nu = 4;
app.Ts = app.dt;
app.p = app.simulation_step;
app.prediction_h = 6;
app.control_h = 4;
app.alpha = 0.4;
app.beta = 0.6;
app.input_max = 0.1;
clf;
for ct = 1:app.agent_num
    app.mpc.agent(ct).data.ny = app.ny;
    app.mpc.agent(ct).data.nx = app.nx;
    app.mpc.agent(ct).data.nu = app.nu;
    app.mpc.agent(ct).data.Ts = app.Ts;
    app.mpc.agent(ct).data.p =  app.p;
    app.mpc.agent(ct).prediction_h = app.prediction_h;
    app.mpc.agent(ct).control_h = app.control_h;
    app.mpc.agent(ct).data.alpha = app.alpha;
    app.mpc.agent(ct).data.beta = app.beta;
    app.mpc.agent(ct).data.input_max = app.input_max;
    
    app.mpc.agent(ct).data.nlobj = nlmpc(app.nx,app.ny,app.nu);
    app.mpc.agent(ct).data.nlobj.Model.StateFcn = "FlyingRobotStateFcn";
    app.mpc.agent(ct).data.nlobj.Jacobian.StateFcn = @FlyingRobotStateJacobianFcn;
    app.mpc.agent(ct).data.nlobj.Ts = app.mpc.agent(ct).data.Ts;
    app.mpc.agent(ct).data.nlobj.PredictionHorizon = app.mpc.agent(ct).data.p;
    app.mpc.agent(ct).data.nlobj.ControlHorizon = app.mpc.agent(ct).data.p;
    
    % (2a), (2b)
    app.mpc.agent(ct).data.nlobj.Optimization.CustomCostFcn = @(X,U,e,data) app.mpc.agent(ct).data.alpha * ((app.mpc.agent(ct).data.Ts * sum(sum(U(1:app.mpc.agent(ct).data.p,:))))^2) + app.mpc.agent(ct).data.beta * ((app.mpc.agent(ct).data.Ts * sum(sum(X(1:app.mpc.agent(ct).data.p,:)-data.References)))^2);
    app.mpc.agent(ct).data.nlobj.Optimization.ReplaceStandardCost = true;
    % final state constraint
    app.mpc.agent(ct).data.nlobj.Optimization.CustomEqConFcn = @(X,U,data) X(end,:)';
    
    for i = 1:app.mpc.agent(ct).data.nu
        app.mpc.agent(ct).data.nlobj.MV(i).Min = 0;
        app.mpc.agent(ct).data.nlobj.MV(i).Max = 1;
    end
    %     app.mpc.agent(ct).data.x0 = [-10 -10 pi/2 0 0 0]';
    %     app.mpc.agent(ct).data.u0 = zeros(app.mpc.agent(ct).data.nu,1);
    app.mpc.agent(ct).data.x0 = app.initial_states(:,ct)';
    app.mpc.agent(ct).data.u0 = zeros(app.mpc.agent(ct).data.nu,1);
    app.mpc.agent(ct).data.x_ref = app.states_ref(:,ct)';
    
    fprintf("validating \n");
    validateFcns(app.mpc.agent(ct).data.nlobj, app.mpc.agent(ct).data.x0, app.mpc.agent(ct).data.u0);
    fprintf("validating end \n");
    
    disp(app.mpc.agent(ct).data.x_ref);
    if generate_trajectory == 1
        [~,~,app.mpc.agent(ct).data.info] = nlmpcmove(app.mpc.agent(ct).data.nlobj, app.mpc.agent(ct).data.x0, app.mpc.agent(ct).data.u0, app.mpc.agent(ct).data.x_ref);
        
        figure(1101);
        subplot(2,4,ct);
        FlyingRobotPlotPlanning(app.mpc.agent(ct).data.info, [1/ct 1/ct 0.1*ct]); drawnow;
    end
    app.mpc.agent(ct).data.nlobj_tracking = nlmpc(app.nx,app.ny,app.nu);
    app.mpc.agent(ct).data.nlobj_tracking.Model.StateFcn = app.mpc.agent(ct).data.nlobj.Model.StateFcn;
    app.mpc.agent(ct).data.nlobj_tracking.Jacobian.StateFcn = app.mpc.agent(ct).data.nlobj.Jacobian.StateFcn;
    
    app.mpc.agent(ct).data.nlobj_tracking.Ts = app.mpc.agent(ct).data.Ts;
    app.mpc.agent(ct).data.nlobj_tracking.PredictionHorizon = app.prediction_h;
    app.mpc.agent(ct).data.nlobj_tracking.ControlHorizon = app.control_h;
    
    app.mpc.agent(ct).data.nlobj_tracking.Weights.ManipulatedVariablesRate = 0.2*ones(1,app.nu);
    app.mpc.agent(ct).data.nlobj_tracking.Weights.OutputVariables = 5*ones(1,app.nx);
    
    % input constraint
    for i = 1:app.nu
        app.mpc.agent(ct).data.nlobj_trackin.MV(i).Min = 0;
        app.mpc.agent(ct).data.nlobj_trackin.MV(i).Max = 0.1;
    end
    
    app.mpc.agent(ct).data.nlobj_trackin.Optimization.CustomEqConFcn = ...
        @(X,U,data) [U(1:end-1,1).*U(1:end-1,2); U(1:end-1,3).*U(1:end-1,4)];
    
    validateFcns(app.mpc.agent(ct).data.nlobj_tracking,app.mpc.agent(ct).data.x0,app.mpc.agent(ct).data.u0);
    
    DStateFcn = @(xk,uk,Ts) FlyingRobotStateFcnDiscreteTime(xk,uk,Ts);
    
    DMeasFcn = @(xk) xk(1:3);
    
    app.mpc.agent(ct).data.EKF = extendedKalmanFilter(DStateFcn,DMeasFcn,app.mpc.agent(ct).data.x0);
    app.mpc.agent(ct).data.EKF.MeasurementNoise = 0.01;
    
    app.mpc.agent(ct).data.xHistory = app.mpc.agent(ct).data.x0;
    app.mpc.agent(ct).data.uHistory = [];
    app.mpc.agent(ct).data.lastMV = zeros(app.nu,1);
    
    app.mpc.agent(ct).data.options = nlmpcmoveopt;
    %% 마지막 2개를 추가하고 0으로 보낸당
    
    if generate_trajectory == 1
        app.mpc.agent(ct).data.Xopt = app.mpc.agent(ct).data.info.Xopt;
        app.mpc.agent(ct).data.Xref = [app.mpc.agent(ct).data.Xopt(2:app.p+1,:);repmat(app.mpc.agent(ct).data.Xopt(end,:),(app.simulation_step + 2)-app.p,1)];
    end
end

fprintf("-------------------------------------------\n");
fprintf("-------------------END---------------------\n");
fprintf("-------------------------------------------\n");













