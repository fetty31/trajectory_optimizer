%% path
addpath('/home/fetty/Escritorio/LaptimeSimulator/LapTime_2021_22/CASADI');


%% import
import casadi.*

%% Options
options.ipopt.print_level = 5;%
%{
options.ipopt.max_iter = 20000;%
options.ipopt.mu_init = 1e15;
options.ipopt.recalc_y = 'yes';
options.ipopt.derivative_test='second-order';
options.ipopt.acceptable_compl_inf_tol = 0.01;
options.ipopt.acceptable_constr_viol_tol = 0.01;
options.ipopt.acceptable_iter = 1000;
options.ipopt.acceptable_obj_change_tol = 0.001;
options.ipopt.acceptable_tol = 1e-2;%
options.ipopt.tol = 1e-3;%
options.ipopt.constraint_violation_norm_type = '2-norm';
options.ipopt.s_max = 100;
%}
%% NLP
%{
minimize: x²+100z²
subject to: z+(1-x)²-y=0
%}

%formulate NLP
x = SX.sym('x');
y = SX.sym('y');
z = SX.sym('z');
f = x^2+100*z^2;
g = z+(1-x)^2-y;
P = struct('f',f,'g',g,'x',[x;y;z]);

%create solver instance
F = nlpsol('F','ipopt',P,options);

%solve problem
r = F('x0',[2.5 3.0 0.75],'ubg',0,'lbg',0); %x0 = initial guess
disp(r.x)


