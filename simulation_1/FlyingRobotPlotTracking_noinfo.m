function FlyingRobotPlotTracking(Ts,Psteps,Tsteps,Xcl,Ucl,index)
% FlyingRobotPlotTracking displays the optimal trajectory of the
% flying robot.
global app

% Copyright 2018 The MathWorks, Inc.
% Xopt = info.Xopt;
tp = Ts*(0:Psteps);
tt = Ts*(0:Tsteps);
% figure(4)
% states = {'x1','x2','theta','v1','v2','omega'};
% for i = 1:6
%     subplot(3,2,i)
%     plot(tt,Xcl(:,i),'+',tp,Xopt(:,i),'-')
%     legend('actual','plan','location','northwest')
%     title(states{i})
% end
% figure(5)
% for i = 1:4
%     subplot(4,1,i)
%     stairs(tt(1:end-1),Ucl(:,i))
%     title(sprintf('Thrust u(%i)', i));
%     axis([0 tt(end) -0.1 1.1])
%     hold on
%     stairs(tp(1:end-1),info.MVopt(1:end-1,i))
%     legend('actual','plan')
%     hold off
% end
figure(6);
subplot(2,4,index);
hold on
% for ct=1:size(Xopt,1)
%     lf = [cos(atan(0.5)+Xopt(ct,3))*0.5 sin(atan(0.5)+Xopt(ct,3))*0.5];
%     rf = [cos(atan(-0.5)+Xopt(ct,3))*0.5 sin(atan(-0.5)+Xopt(ct,3))*0.5];
%     lr = [cos(pi-atan(0.5)+Xopt(ct,3))*0.5 sin(pi-atan(0.5)+Xopt(ct,3))*0.5];
%     rr = [cos(pi-atan(-0.5)+Xopt(ct,3))*0.5 sin(pi-atan(-0.5)+Xopt(ct,3))*0.5];
% %     patch([lf(1) rf(1) rr(1) lr(1)]+Xopt(ct,1),[lf(2) rf(2) rr(2) lr(2)]+Xopt(ct,2),'y','FaceAlpha',0.5,'LineStyle',':');
%     hold on
% end
for ct=1:size(Xcl,1)
    lf = [cos(atan(0.5)+Xcl(ct,3))*0.5 sin(atan(0.5)+Xcl(ct,3))*0.5];
    rf = [cos(atan(-0.5)+Xcl(ct,3))*0.5 sin(atan(-0.5)+Xcl(ct,3))*0.5];
    lr = [cos(pi-atan(0.5)+Xcl(ct,3))*0.5 sin(pi-atan(0.5)+Xcl(ct,3))*0.5];
    rr = [cos(pi-atan(-0.5)+Xcl(ct,3))*0.5 sin(pi-atan(-0.5)+Xcl(ct,3))*0.5];
    if ct<size(Xcl,1)
        patch([lf(1) rf(1) rr(1) lr(1)]+Xcl(ct,1),[lf(2) rf(2) rr(2) lr(2)]+Xcl(ct,2),'b','FaceAlpha',0.5);
        hold on
    else
        patch([lf(1) rf(1) rr(1) lr(1)]+Xcl(ct,1),[lf(2) rf(2) rr(2) lr(2)]+Xcl(ct,2),'r','FaceAlpha',0.5);
        hold on
    end
end
xlabel('x')
ylabel('y')
xlim([-11 10]);
ylim([-11 10]);
title_name = num2str(index);
title_name = strcat("Robot", title_name);
title_name = strcat(title_name, " Trajectory");
ref_x = num2str(app.states_ref(1,index));
ref_y = num2str(app.states_ref(2,index));
start_x = num2str(app.initial_states(1,index));
start_y = num2str(app.initial_states(2,index));
information = ['   ref: (' ref_x ', ' ref_y '),,,,start: (' start_x ', ' start_y ')'];
title_name = strcat(title_name, information);
title(title_name)
% fprintf('index : %d , Actual fuel consumption = %10.6f\n',index ,sum(sum(Ucl(1:end-1,:)))*Ts);
