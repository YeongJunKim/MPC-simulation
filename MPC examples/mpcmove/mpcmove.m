ts = 0.1;

% state space model

Plant = ss(0.8,0.5,0.25,0,ts);
MPCobj = mpc(Plant);

MPCobj.MV(1).Min = -2;
MPCobj.MV(1).Max = 2;

x = mpcstate(MPCobj);

r = 1;

t = [0:ts:40];
N = length(t);
y = zeros(N,1); 
u = zeros(N,1); 
for i = 1:N
    % simulated plant and predictive model are identical
    y(i) = 0.25*x.Plant;
    u(i) = mpcmove(MPCobj,x,y(i),r);
end

[ts,us] = stairs(t,u);
plot(ts,us,'r-',t,y,'b--')
legend('MV','OV')

MPCopt = mpcmoveopt;
MPCopt.MVMin = -2;
MPCopt.MVMax = 2;

x = mpcstate(MPCobj);
y = zeros(N,1);
u = zeros(N,1);
for i = 1:N
    % simulated plant and predictive model are identical
    y(i) = 0.25*x.Plant;
    if i == 5
    	MPCopt.MVMax = 1;
    end
    u(i) = mpcmove(MPCobj,x,y(i),r,[],MPCopt);
end

[ts,us] = stairs(t,u);
plot(ts,us,'r-',t,y,'b--')
legend('MV','OV')














