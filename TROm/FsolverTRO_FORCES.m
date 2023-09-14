clear all
close all
%% Path
addpath('/home/fetty/FORCES');
addpath('/home/fetty/Escritorio/LaptimeSimulator/LapTime_2021_22/CASADI');
%addpath('/home/fetty/Escritorio/LaptimeSimulator/TRO/params_albert');

%% Load
load('EnduranceSpain.mat')
gro_track = readtable('gro_0.0490.csv');

%% Problem dimensions
model.N = 80; %horizon length
model.nvar = 9; %number of variables
model.neq = 7; %number of equality constraints
model.nh = 2; %number of inequality constraint functions
model.npar = 23; %number of real time parametres 

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
%model.xinitidx = 1:model.nvar;
model.xfinalidx = [3,4,5,6,7,8,9]; %[3,4,5,6,7,8,9];

%% Define solver options
codeoptions = getOptions('finalCURVATUREsolver');
codeoptions.maxit = 1000; % Maximum number of iterations
codeoptions.printlevel = 1; 
codeoptions.optlevel = 0;
% 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.cleanup = false;
codeoptions.timing = 1;
codeoptions.overwrite = 1;
codeoptions.BuildSimulinkBlock = 0;
codeoptions.platform = 'Gnu-x86_64';

% Necessary to set bounds dynamically
codeoptions.nlp.stack_parambounds = 1;
%codeoptions.noVariableElimination = 1; 
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

%% SIMULATION

delta_s = 0.1; %discretization space [m]
%param_data = textread('all_params.txt', '%f', 'delimiter', '\n');

param_data = [];
for i=1:2:length(gro_track.curvature(1:end))
    k = gro_track.curvature(i);
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

lb = repmat(z_lb,model.N,1);
ub = repmat(z_ub,model.N,1);

problem.lb = lb(1:end-7,:); %if xinit is given -- lb(1:end,:)
problem.ub = ub(1:end-7,:); %                  -- ub(1:end,:)

%inequality constraints with friction circle
%problem.hu = repmat([1.5;1.5;(Dr*lambda)^2;(Df*lambda)^2],model.N,1);
%problem.hl = repmat([0.001;0.001;0.001;0.001],model.N,1);

%inequality constraints 
problem.hu = repmat([1.5;1.5],model.N,1);
problem.hl = repmat([0.001;0.001],model.N,1);

%xinit = [0.000421622568141444;-0.0109715182977683;-0.0534936937978844;4.33929859768719;...
%    -0.383778279211991;0.0278299955734653;29.9999997291638;1.97435258322106;0.767947757861118];

%xfinal = [0.00206572774511192;-0.00263645687860950;-0.0549031831756352;4.76284396374887;...
%    0.701205844717989;-0.00211485268894678;29.9999997948522;1.78973789402130;0.604984723777449];

xfinal = [0.00645986891685226;0.0199862268886348;0.0164610333264601;4.70391229177228;0.701990878004544;-0.00169343472216051;29.9998248521465;1.85516033103700;1.09693022953308];

xinit = [0.000550136548175241;-0.00328416699755482;-0.0501471830337785;4.16388475782800;0.607114718258554;0.00679374547205829;29.9999997577289;1.93519869067698;0.892605545710788];

x0 = lb +(ub-lb)/2;
%problem.x0 = repmat(xinit,model.N,1);
problem.x0 = x0;

%problem.xinit = xinit;
problem.xfinal = xfinal(3:end);
%problem.x0 = repmat(xinit,model.N,1);

kant = 1673; %4145;

problem.all_parameters = reshape(param_data(:,kant:kant+model.N-1),model.npar*model.N,1);

[output,exitflag,info] = finalCURVATUREsolver(problem);
exitflag


%% Solver functions

function f = objective(z,p)

    delta_d = z(1);
    delta_a = z(2);
    steering = z(1) + z(3);
    n = z(5);
    mu = z(6);
    vx = z(7);
    vy = z(8);
    w = z(9);
    
    Lf = p(5);
    Lr = p(6);
    dRa = p(1);
    dRd = p(2);
    %dRa = 0.3;
    %dRd = 0.7;
    q_slip = p(18);
    %q_slip = 0.1;
    k = p(20);
    q_n = p(21);
    q_mu = p(22);
    
    % Progress rate minimization term
    sdot = (vx*cos(mu) - vy*sin(mu))/(1 - n*k);
    
    % Slip difference
    beta_dyn = atan(vy/vx);
    beta_kin = atan(steering*Lr/(Lr+Lf));
    diff_beta = beta_dyn - beta_kin;
    
    %f = -sdot + (delta_d)^2*dRd + (delta_a)^2*dRa + q_slip*(diff_beta)^2;
    %f = (delta_d)^2*dRd + (delta_a)^2*dRa + q_slip*(diff_beta)^2;
    f = -sdot;
    
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
    Fx = m*a - m*u_r*g - 0.5*Cd*rho*Ar*vx^2; 
    
    %Term to change from time dependent to space dependent -- (Ft(u,x)/sdot = Fs(u,x))
    sdot = (vx*cos(mu) - vy*sin(mu))/(1 - n*k);
    
    % Differential equations (now space dependent)
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
    integrator_stepsize = 0.1; %space step -- 0.1m
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
    
    %Fe_r = (p_long*Fx)^2 + Fr^2; %Friction Elipse
    %Fe_f = (p_long*Fx)^2 + Ff^2;
    
    h = [n - length/2*sin(abs(mu)) + width/2*cos(mu);
    -n + length/2*sin(abs(mu)) + width/2*cos(mu)];
    %Fe_r; Fe_f];
end

