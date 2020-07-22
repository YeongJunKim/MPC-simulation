function mv=mpc_run(index, x, u, x_ref)
global app
options = nlmpcmoveopt;
[mv,opt,app.mpc.agent(index).data.info] = nlmpcmove(app.mpc.agent(index).data.nlobj, x, u, x_ref,[], options);
disp("mv");
disp(mv);
% disp(mv);
% disp(opt);
% disp(app.mpc.agent(index).data.info);
% disp(mv)
% disp(opt)
