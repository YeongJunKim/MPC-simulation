global app

for ct = 1:app.agent_num
    ny = 3;
    nx = 3;
    nu = 2;
    Ts = 0.1;
    p = 10;
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
    app.mpc.agent(ct).data.nlobj.Model.StateFcn = "AgentStateFcn";
    % And jacobian function of kinematics
    app.mpc.agent(ct).data.nlobj.Jacobian.StateFcn = @AgentStateJacobianFcn;
    % sampling time, hrizonsize, PredictionHorizion
    app.mpc.agent(ct).data.nlobj.Ts = Ts;
    app.mpc.agent(ct).data.nlobj.PredictionHorizon = p;
    app.mpc.agent(ct).data.nlobj.ControlHorizon = 3;
    
    
    app.mpc.agent(ct).data.nlobj.Weights.ManipulatedVariablesRate = 0.2*ones(1,nu);
    app.mpc.agent(ct).data.nlobj.Weights.OutputVariables = 5*ones(1,nx);
    
%     for i = 1:nu
%        app.mpc.agent(ct).data.nlobj.MV(i).Min = 0;
%        app.mpc.agent(ct).data.nlobj.MV(i).Max = 1;
%     end
    % linear and angular velocity constraint
    app.mpc.agent(ct).data.nlobj.MV(1).Min = -0.22;
    app.mpc.agent(ct).data.nlobj.MV(1).Max = 0.22;
    app.mpc.agent(ct).data.nlobj.MV(2).Min = -2.84;
    app.mpc.agent(ct).data.nlobj.MV(2).Max = 2.84;
    
    
%     app.mpc.agent(ct).data.x0 = [-10 -10 pi/2 0 0 0]';
%     app.mpc.agent(ct).data.u0 = zeros(nu,1);
    app.mpc.agent(ct).data.x0 = app.initial_states(:,ct);
    app.mpc.agent(ct).data.u0 = zeros(nu,1);

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
    
    
end















