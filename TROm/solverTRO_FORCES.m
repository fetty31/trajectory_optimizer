clear all
close all
%% Path
addpath('/home/fetty/FORCES');
addpath('/home/fetty/Escritorio/LaptimeSimulator/LapTime_2021_22/CASADI');
%addpath('/home/fetty/Escritorio/LaptimeSimulator/TRO/params_albert');

%% Problem dimensions
model.N = 328; %horizon length %342 
model.nvar = 9; %n variables
model.neq = 7; %n equality constraints
model.nh = 2; %n inequality constraints
model.npar = 24; %n real-time parametres 
delta_s = 0.4897; %discretization space [m] (if changed -- change integrator_stepsize from RK4 method!)

%% Objective function
model.objective = @objective;

%% Matrix equality constraints

%z = [delta_d, delta_a, delta, a, n, mu, vx, vy, w]

model.eq = @dynamics;
model.E = [zeros(7, 2), eye(7)];

%% Variable bounds
% upper/lower bounds lb <= z <= ub
model.lb = [];
model.ub = [];
model.lbidx = (1:model.nvar)';
model.ubidx = (1:model.nvar)';

%% Inequalities 
% hl <= h(z) <= hu
model.ineq = @inequalities;
model.hu = [];
model.hl = [];
model.huidx = (1:model.nh)';
model.hlidx = (1:model.nh)';

%% Initial and final conditions
%model.xinitidx = [1,3,5,6,8,9];
%model.xfinalidx = [3,4,5,6,7,8,9];

%% Define solver options
codeoptions = getOptions('CURVATUREsolver');
codeoptions.maxit = 5000; % Maximum number of iterations
codeoptions.printlevel = 0; 
codeoptions.optlevel = 0;
% 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.cleanup = false;
codeoptions.timing = 1;
codeoptions.overwrite = 1;
codeoptions.BuildSimulinkBlock = 0;
codeoptions.platform = 'Gnu-x86_64';
%codeoptions.noVariableElimination = 1;

% Necessary to set bounds dynamically
codeoptions.nlp.stack_parambounds = 1;
%codeoptions.dump_formulation = 1;

% Default tolerances
codeoptions.nlp.TolStat = 1e-5; %inf norm tol. on stationarity
codeoptions.nlp.TolEq = 1e-6;
codeoptions.nlp.TolIneq = 1e-6;
codeoptions.nlp.TolComp = 1e-6;

% Speed up the solver
%codeoptions.nlp.checkFunctions = 0;
%codeoptions.nlp.linear_solver = 'symm_indefinite_fast';

%% Generate forces solver
FORCES_NLP(model, codeoptions);

%{
%% SIMULATION

delta_s = 0.1; %discretization space [m]
%param_data = textread('all_params.txt', '%f', 'delimiter', '\n');

param_data = [];
for i=1:2:length(track.r(1:end))
    k = track.r(i);
    param_k = [.5, .5, 240, 93, .708, .822, 3152.3, 2785.4, 1.6, 1.6, 10.1507, 10.8529, .45, 9.81, .8727, 1.255, 1, .1, 0.5, k, .25, 0, 2]';
    %param_data = [param_data;param_k];
    param_data = [param_data,param_k];
end
param_data = repmat(param_data,1,2);
%x0 = textread('x0.txt', '%f', 'delimiter', '\n');

% Parameters for upper bounds of Friction Elipse constrain
Dr = param_data(7);
Df = param_data(8);
lambda = param_data(23);

% Upper and lower bounds of the problem (variables + constrains)
z_lb = [-deg2rad(40)*delta_s,-10*delta_s,-deg2rad(23),-10,-2,-deg2rad(40),0,-2,-2*pi]';
z_ub = [deg2rad(40)*delta_s,10*delta_s,deg2rad(23),10,2,deg2rad(40),30,2,2*pi]';

%problem.all_parameters = param_data;

problem.lb = repmat(z_lb,model.N,1);
problem.ub = repmat(z_ub,model.N,1);

%problem.hu = repmat([1.5;1.5;(Dr*lambda)^2;(Df*lambda)^2],model.N,1);
%problem.hl = repmat([0.001;0.001;0.001;0.001],model.N,1);
problem.hu = repmat([1.5;1.5],model.N,1);
problem.hl = repmat([0.001;0.001],model.N,1);

x0 = [0,0,0,0,1,0.2,10,0.2,0]';
%x0 = zeros(9,1);
problem.x0 = repmat(x0,model.N,1);

%problem.xinit = textread('xinit.txt', '%f', 'delimiter', '\n');
xinit = zeros(9,1);
%problem.xinit = xinit;
%problem.xfinal = [];

%OUTPUT = [];
X_OPT = [];
FLAGS = [];
OUTPUT = struct;
kant = 1; k = 1;
smax = round(track.x(end),1);
tcontrol = 4;
s = 0;
S = [s];
tic

while s<smax
    
    problem.all_parameters = reshape(param_data(:,kant:kant+model.N-1),model.npar*model.N,1);
    problem.x0 = repmat(x0,model.N,1);
    
    [output,exitflag,info] = CURVATUREsolver(problem);
    %OUTPUT = [OUTPUT;full(output)];
    FLAGS(k) = exitflag;
    if exitflag == 1
        X_OPT = [X_OPT,full(output.x04)];
        x0 = output.x04;
        if s==0
            x0init = x0;
        end
        %sdot = (x0(7)*cos(x0(6)) - x0(8)*sin(x0(6)))/(1 - x0(5)*problem.all_parameters(model.npar*tcontrol-3));
        s = s + delta_s*tcontrol;
        S = [S,s];
    end
    if abs(s-smax)<1
        problem.xfinal = x0init;
    end
       
    kant = kant+tcontrol;
    k = k+1;
end
plot(S(1:end-1),X_OPT(7,:));
hold on
figure()
plot(track.X,track.Y);
hold on
plot(track.X(1),track.Y(1),'o','Color','r');
hold off
%{
kant = 1;
for k=1:693/33
    problem.all_parameters = repmat(param_data(kant:k*model.npar,1),model.N,1);
    % Time to solve the NLP!
    [output,exitflag,info] = CURVATUREsolver(problem);
    FLAGS(k) = exitflag;
    
    kant = k*model.npar+1;
end
%}
toc

%}
%% SIMULATION
gro_track = readtable('gro_0.0490_1.csv');
%load('EnduranceSpain.mat');
load('TrackTFGDef.mat');

%param_data = textread('all_params.txt', '%f', 'delimiter', '\n');
curvature = [];
param_data = [];
for i=1:10:length(track.r)%(gro_track.curvature)
    k = track.r(i);
    curvature = [curvature,k];
    %param = [dRd, dRa, m, I, Lf, Lr, Dr, Df, Cr, Cf, Br, Bf, u_r, g, Cd, rho, Ar, q_slip, p_long, k, q_n, q_mu, lambda, q_s]
    param_k = [.5, .3, 240, 93, .708, .822, 3152.3, 2785.4, 1.6, 1.6, 10.1507, 10.8529, .45, 9.81, .8727, 1.255, 1, .1, 0.5, k, 0, 0.5, 2, 2]';
    %param_data = [param_data;param_k];
    param_data = [param_data,param_k];
end
%param_data = repmat(param_data,1,2);
%x0 = textread('x0.txt', '%f', 'delimiter', '\n');

% Parameters for upper bounds of Friction Elipse constrain
Dr = param_data(7);
Df = param_data(8);
lambda = param_data(23);

% Upper and lower bounds of the problem (variables + constrains)
z_lb = [-deg2rad(3),-5,-deg2rad(23),-10,-2,-deg2rad(100),0,-2,-18.85]';
z_ub = [deg2rad(3),.25,deg2rad(23),6.5,2,deg2rad(100),30,2,18.85]';

%lb1 = [-10,-10,0]';
%ub1 = [10,10,30]';

lb = repmat(z_lb,model.N,1);
ub = repmat(z_ub,model.N,1);

%lb = [lb(1:2);lb(10:end)];
%ub = [ub(1:2);ub(10:end)];

%problem.lb = [lb1;lb];
%problem.ub = [ub1;ub];

problem.lb = lb(1:end);
problem.ub = ub(1:end);

%inequality constraints with friction circle
%problem.hu = repmat([1.5;1.5;(Dr*lambda)^2;(Df*lambda)^2],model.N,1);
%problem.hl = repmat([0.001;0.001;0.001;0.001],model.N,1);

%inequality constraints 
problem.hu = repmat([1.5;1.5],model.N,1);
problem.hl = repmat([0.001;0.001],model.N,1);

%lb = repmat(z_lb,model.N,1);
%ub = repmat(z_ub,model.N,1);
x0 = lb +(ub-lb)/2;
problem.x0 = x0;
%x0 = [0,0,0,0,0,0.01,15,0,0]';
%problem.x0 = repmat(x0,model.N,1);

%problem.xinit = zeros(6,1);
%problem.xinit = [0,0,0.001,0.01,0,0]';
%problem.xfinal = x0(3:9);

%problem.all_parameters = reshape(param_data,model.npar*model.N,1);
kant = 1;
problem.all_parameters = reshape(param_data(:,kant:kant+model.N-1),model.npar*model.N,1);

[output,exitflag,info] = CURVATUREsolver(problem);
exitflag

X_OPT = [x0(1:9),tf_output(output,model.N)];
S = 0:delta_s:delta_s*model.N;

figure()
subplot(2,2,1)
plot(S,X_OPT(7,:));
xlabel('S [m]');
ylabel('Vx [m/s]');

subplot(2,2,2)
plot(S,X_OPT(3,:))
xlabel('S [m]');
ylabel('delta [rad]');

subplot(2,2,3)
plot(S,X_OPT(6,:))
xlabel('S [m]');
ylabel('mu [rad]');

subplot(2,2,4)
plot(S,X_OPT(5,:))
xlabel('S [m]');
ylabel('n [m]');
hold off
%% Solver functions

function f = objective(z,p)

    delta_d = z(1);
    delta_a = z(2);
    steering = delta_d + z(3);
    
    n = z(5);
    mu = z(6);
    vx = z(7);
    vy = z(8);
    w = z(9);
    
    Lf = p(5);
    Lr = p(6);
    dRa = p(2);
    dRd = p(1);
    q_slip = p(18);
    k = p(20);
    q_n = p(21);
    q_mu = p(22);
    q_s = p(24);
    
    % Progress rate minimization term
    sdot = (vx*cos(mu) - vy*sin(mu))/(1 - n*k);
    
    % Slip difference
    beta_dyn = atan(vy/vx);
    beta_kin = atan(steering*Lr/(Lr+Lf));
    diff_beta = beta_dyn - beta_kin;
    
    f = -q_s*sdot + (delta_d)^2*dRd + (delta_a)^2*dRa + q_slip*(diff_beta)^2 + q_mu*(mu)^2 ;%+ q_n*(n)^2;
    %f = -q_s*sdot + (delta_a)^2*dRa + (delta_d)^2*dRd + q_slip*(diff_beta)^2;

end

function xdot = continuous_dynamics(x, u, p)
    % Real time parameters used in the dynamic model
    m = p(3);
    I = p(4);
    Lf = p(5);
    Lr = p(6);
    Dr = p(7);
    Df = p(8);
    Cr = p(9);
    Cf = p(10);
    Br = p(11);
    Bf = p(12); 
    u_r = p(13);
    g = p(14);
    Cd = p(15);
    rho = p(16);
    Ar = p(17);
    k = p(20);

    % States
    n = x(1);
    mu = x(2);
    vx = x(3);
    vy = x(4);
    w = x(5);
    a = u(2);
    delta = u(1);
    
    % Slip angles
    alpha_R = atan((vy-Lr*w)/(vx));
    alpha_F = atan((vy+Lf*w)/(vx)) - delta;

    % Pacejka lateral force magic formula
    Fr = Dr*sin(Cr*atan(Br*alpha_R));
    Ff = Df*sin(Cf*atan(Bf*alpha_F));
    Fx = m*a - m*u_r*g - 0.5*Cd*rho*Ar*vx^2; %m*a*(1+cos(delta))
    
    %Term to change from time dependent to space dependent -- (Ft(u,x)/sdot = Fs(u,x))
    sdot = (vx*cos(mu) - vy*sin(mu))/(1 - n*k);
    
    % Differential equations (now space dependent)
    %{
    xdot = [vx*sin(mu) + vy*cos(mu)/sdot;
            w/sdot - k;
            (1/m)*(Fx - Ff*sin(delta) + m*vy*w)/sdot;
            (1/m)*(Fr + a*sin(delta) + Ff*cos(delta) - m*vx*w)/sdot;
            (1/I)*(a*sin(delta)*Lf + Ff*Lf*cos(delta) - Fr*Lr)/sdot];
    %}
    xdot = [vx*sin(mu) + vy*cos(mu)/sdot;
            w/sdot - k;
            (1/m)*(Fx - Ff*sin(delta) + m*vy*w)/sdot;
            (1/m)*(Fr + Ff*cos(delta) - m*vx*w)/sdot;
            (1/I)*(Ff*Lf*cos(delta) - Fr*Lr)/sdot];
end

function xnext = dynamics(z,p)
    u = z(1:2);
    x = z(3:9);
    U = u + x(1:2);
    % implements a RK4 integrator for the dynamics
    integrator_stepsize = 0.4897; %space step -- 0.5m
    next_state = RK4( x(3:end), U, @continuous_dynamics, integrator_stepsize, p);
    xnext = [U; next_state];
end

function h = inequalities(z,p)
    
    % Real time parameters used in the dynamic model
    Lf = p(5);
    Lr = p(6);
    m = p(3);
    Dr = p(7);
    Df = p(8);
    Cr = p(9);
    Cf = p(10);
    Br = p(11);
    Bf = p(12); 
    u_r = p(13);
    g = p(14);
    Cd = p(15);
    rho = p(16);
    Ar = p(17);
    p_long = p(19);
    lambda = p(23);

    % States
    n = z(5);
    mu = z(6);
    vx = z(7);
    vy = z(8);
    w = z(9);
    a = z(2);
    delta = z(1);
    
    % Length and width of the car
    length = 2.72;
    width = 1.2 + 0.4;
    
    % Slip angles
    alpha_R = atan((vy-Lr*w)/(vx));
    alpha_F = atan((vy+Lf*w)/(vx)) - delta;
    
    % Pacejka lateral force magic formula
    Fr = Dr*sin(Cr*atan(Br*alpha_R));
    Ff = Df*sin(Cf*atan(Bf*alpha_F));
    Fx = m*a - m*u_r*g - 0.5*Cd*rho*Ar*vx^2; 
    
    %Fe_r = (p_long*Fx)^2/(lambda*Dr)^2 + (Fr)^2/(lambda*Dr)^2; %Friction Elipse
    %Fe_f = (p_long*Fx)^2/(lambda*Df)^2 + (Ff)^2/(lambda*Df)^2;
    
    Fe_r = (p_long*Fx)^2 + Fr^2; %Friction Elipse
    Fe_f = (p_long*Fx)^2 + Ff^2;
    
    h = [n - length/2*sin(abs(mu)) + width/2*cos(mu);
    -n + length/2*sin(abs(mu)) + width/2*cos(mu)];
    %;Fe_r; Fe_f];
end

