function dxdt = AgentStateFcn(x, u)
    % state
    % x(1) = x cordinate
    % x(2) = y cordinate
    % x(3) = theta, direction from heading angle
    % input
    % u(1) = 
    dxdt = zeros(3,1);
    dxdt(1) = cos(x(3))*u(1);
    dxdt(2) = sin(x(3))*u(1);
    dxdt(3) = u(2);
end