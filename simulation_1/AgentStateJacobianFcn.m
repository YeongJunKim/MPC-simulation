function [A, B] = AgentStateJacobianFcn(x, u)


A = zeros(3,3);
A(1,3) = -sin(x(3))*u(1);
A(2,3) = cos(x(3))*u(1);
A(3,3) = u(2);

B = zeros(3,2);
B(1,1) = cos(x(3));
B(2,1) = sin(x(3));
B(3,2) = 1;



%% test

% syms x y theta
% syms v_l v_theta
% syms pj
% f.x(x,y,theta,v_l,v_theta) = v_l * cos(theta);
% f.y(x,y,theta,v_l,v_theta) = v_l * sin(theta);
% f.theta(x,y,theta,v_l,v_theta) = theta + v_theta;
% f = [f.x f.y f.theta]'; jf = jacobian(f, [x,y,theta]);
% f = matlabFunction(f); jf = matlabFunction(jf);


% fu = [f.x f.y f.theta]'; jfu = jacobian(fu, [v_l v_theta]);
% fu = matlabFunction(fu); jfu = matlabFunction(jfu);
%%