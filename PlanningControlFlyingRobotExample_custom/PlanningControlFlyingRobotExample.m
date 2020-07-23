close all
nx = 6;
ny = 6;
nu = 4;
nlobj = nlmpc(nx,ny,nu);

nlobj.Model.StateFcn = "FlyingRobotStateFcn";

nlobj.Jacobian.StateFcn = @FlyingRobotStateJacobianFcn;

Ts = 0.4;
p = 20;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;

nlobj.ControlHorizon = p;

nlobj.Optimization.CustomCostFcn = @(X,U,e,data) Ts*sum(sum(U(1:p,:)));

nlobj.Optimization.ReplaceStandardCost = true;

nlobj.Optimization.CustomEqConFcn = @(X,U,data) X(end,:)';

for ct = 1:nu
    nlobj.MV(ct).Min = 0;
    nlobj.MV(ct).Max = 1;
end
x0 = [-10 -10 pi/2 0 0 0]';  % robot parks at [-10, -10], facing north
u0 = zeros(nu,1);           % thrust is zero

fprintf("validate\n");
validateFcns(nlobj,x0,u0);
fprintf("validate end\n");

[~,~,info] = nlmpcmove(nlobj,x0,u0);

FlyingRobotPlotPlanning(info);


%% tracking


nlobj_tracking = nlmpc(nx,ny,nu);

nlobj_tracking.Model.StateFcn = nlobj.Model.StateFcn;
nlobj_tracking.Jacobian.StateFcn = nlobj.Jacobian.StateFcn;

nlobj_tracking.Ts = Ts;
nlobj_tracking.PredictionHorizon = 10;
nlobj_tracking.ControlHorizon = 4;

nlobj_tracking.Weights.ManipulatedVariablesRate = 0.2*ones(1,nu);
nlobj_tracking.Weights.OutputVariables = 5*ones(1,nx);

for ct = 1:nu
    nlobj_tracking.MV(ct).Min = 0;
    nlobj_tracking.MV(ct).Max = 1;
end

nlobj_tracking.Optimization.CustomEqConFcn = ...
    @(X,U,data) [U(1:end-1,1).*U(1:end-1,2); U(1:end-1,3).*U(1:end-1,4)];

validateFcns(nlobj_tracking,x0,u0);

DStateFcn = @(xk,uk,Ts) FlyingRobotStateFcnDiscreteTime(xk,uk,Ts);

DMeasFcn = @(xk) xk(1:3);

EKF = extendedKalmanFilter(DStateFcn,DMeasFcn,x0);
EKF.MeasurementNoise = 0.01;


%% run
Tsteps = 32;        
xHistory = x0';
uHistory = [];
lastMV = zeros(nu,1);

Xopt = info.Xopt;
Xref = [Xopt(2:p+1,:);repmat(Xopt(end,:),Tsteps-p,1)];

%% Use |nlmpcmove| and |nlmpcmoveopt| command for closed-loop simulation.
hbar = waitbar(0,'Simulation Progress');
options = nlmpcmoveopt;
for k = 1:Tsteps
    yk = xHistory(k,1:3)' + randn*0.01;
    xk = correct(EKF, yk);
    [uk,options] = nlmpcmove(nlobj_tracking,xk,lastMV,Xref(k:min(k+9,Tsteps),:),[],options);
    disp("uk");
    disp(uk);
    predict(EKF,uk,Ts);
    uHistory(k,:) = uk';
    lastMV = uk;
    ODEFUN = @(t,xk) FlyingRobotStateFcn(xk,uk);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
    xHistory(k+1,:) = YOUT(end,:);            
    waitbar(k/Tsteps, hbar);
end
close(hbar)

FlyingRobotPlotTracking(info,Ts,p,Tsteps,xHistory,uHistory);


