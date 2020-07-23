function mpc_initialization()

global app
app.ny = 6;
app.nx = 6;
app.nu = 4;
app.Ts = 0.4;
app.p = 30;
for ct = 1:app.agent_num
    ny = 6;
    nx = 6;
    nu = 4;
    Ts = 0.5;
    p = 30;
    
    app.mpc.agent(ct).data.ny = ny;
    app.mpc.agent(ct).data.nx = nx;
    app.mpc.agent(ct).data.nu = nu;
    app.mpc.agent(ct).data.initial_input = zeros(2,1);
    app.mpc.agent(ct).data.spec.robot.alpha = 0.2;
    app.mpc.agent(ct).data.spec.robot.beta = 0.2;
    % specifications https://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/
    % linear & angular velocity
    
    % MPC init
    app.mpc.agent(ct).data.nlobj = nlmpc(nx,ny,nu);
    % State function is nonholonomic kinematics
    app.mpc.agent(ct).data.nlobj.Model.StateFcn = "FlyingRobotStateFcn";
    % And jacobian function of kinematics
    app.mpc.agent(ct).data.nlobj.Jacobian.StateFcn = @FlyingRobotStateJacobianFcn;
    % sampling time, hrizonsize, PredictionHorizion
    app.mpc.agent(ct).data.nlobj.Ts = Ts;
    app.mpc.agent(ct).data.nlobj.PredictionHorizon = p;
    app.mpc.agent(ct).data.nlobj.ControlHorizon = 3;
    
    
    app.mpc.agent(ct).data.nlobj.Weights.ManipulatedVariablesRate = 0.2*ones(1,nu);
    app.mpc.agent(ct).data.nlobj.Weights.OutputVariables = 5*ones(1,nx);
    
    for i = 1:nu
       app.mpc.agent(ct).data.nlobj.MV(i).Min = 0;
       app.mpc.agent(ct).data.nlobj.MV(i).Max = 1;
    end
    % linear and angular velocity constraint
%     app.mpc.agent(ct).data.nlobj.MV(1).Min = -0.22;
%     app.mpc.agent(ct).data.nlobj.MV(1).Max = 0.22;
%     app.mpc.agent(ct).data.nlobj.MV(2).Min = -2.84;
%     app.mpc.agent(ct).data.nlobj.MV(2).Max = 2.84;
    
    
    app.mpc.agent(ct).data.x0 = [-10 -10 pi/2 0 0 0]';
    app.mpc.agent(ct).data.u0 = zeros(nu,1);
%     app.mpc.agent(ct).data.x0 = app.initial_states(:,ct);
%     app.mpc.agent(ct).data.u0 = zeros(nu,1);

    % Optimization.CustomEqConFcn
    % Now I just add fuel constraint
%     app.mpc.agent(ct).data.nlobj.Optimization.CustomCostFcn = @(X,U,e,data) Ts*sum(sum(U(1:p,:)));
    % The goal of the maneuver is converge target
    app.mpc.agent(ct).data.nlobj.Optimization.CustomEqConFcn = @(X,U,data) X(end,:)';
    
    fprintf("validate\n");
    validateFcns(app.mpc.agent(ct).data.nlobj,app.mpc.agent(ct).data.x0',app.mpc.agent(ct).data.u0');
    fprintf("validate end\n");
    
    
    app.mpc.agent(ct).data.nlobj.Optimization.CustomEqConFcn = ...
        @(X,U,data) [U(1:end-1,1).*U(1:end-1,2)];
    
    % I don't know exactly.
    nlobj.Optimization.ReplaceStandardCost = true;
    
    
    % MPC scheme cost function are described in (2a, 2b)
    
    % D-MPC scheme cost function are described in ()
    
    % DC-MPC scheme cost function are described in (3a, 3b)
    
    
    % estimator
    
    app.mpc.agent(ct).data.DStateFcn = @(xk,uk,Ts) FlyingRobotStateFcnDiscreteTime(xk,uk,Ts);
    app.mpc.agent(ct).data.DMeasFcn = @(xk) xk(1:3);
    app.mpc.agent(ct).data.EKF = extendedKalmanFilter(app.mpc.agent(ct).data.DStateFcn,app.mpc.agent(ct).data.DMeasFcn,app.mpc.agent(ct).data.x0);
    app.mpc.agent(ct).data.EKF.MeasurementNoise = 0.01;
    app.mpc.agent(ct).data.xHistory = app.mpc.agent(ct).data.x0;
    app.mpc.agent(ct).data.uHistory = [];
    app.mpc.agent(ct).data.lastMV = zeros(nu,1);
    app.mpc.agent(ct).data.ref = [normrnd(5,3) normrnd(0,4) normrnd(0,0.1), 0, 0, 0]';
    app.mpc.agent(ct).data.ref = [-8 -9 0 0 0 0]';
    app.mpc.agent(ct).data.options = nlmpcmoveopt;
end















