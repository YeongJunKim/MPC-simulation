nx = 6;
ny = 6;
nu = 4;
nlobj = nlmpc(nx,ny,nu);
nlobj.Model.StateFcn = "FlyingRobotStateFcn";
nlobj.Jacobian.StateFcn = @FlyingRobotStateJacobianFcn;

Ts = 0.4;
p = 30;
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

x0 = [-10;-10;pi/2;0;0;0];  % robot parks at [-10, -10], facing north
u0 = zeros(nu,1);           % thrust is zero


validateFcns(nlobj,x0,u0);

[~,~,info] = nlmpcmove(nlobj,x0,u0);

FlyingRobotPlotPlanning(info);




























