clear all
close all
%% Path
addpath('/home/fetty/FORCES');
addpath('/home/fetty/Escritorio/LaptimeSimulator/LapTime_2021_22/CASADI');
addpath('/home/fetty/Escritorio/LaptimeSimulator/TRO/params_albert');

%% Load
load('EnduranceSpain.mat')

%% Problem dimensions
model.N = 40; %horizon length
model.nvar = 9; %number of variables
model.neq = 7; %number of equality constraints
model.nh = 4; %number of inequality constraint functions
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
%model.xinit = [-2, 0, 0, deg2rad(120)]';
%model.xinitidx = 1:model.nvar;
%model.xfinal = [0, deg2rad(0)]';
%model.xfinalidx = [5,6,7,8,9];

%% Define solver options
codeoptions = getOptions('CURVATUREsolver');
codeoptions.maxit = 200; % Maximum number of iterations
codeoptions.printlevel = 5; 
codeoptions.optlevel = 0;
% 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.cleanup = false;
codeoptions.timing = 1;
codeoptions.printlevel = 0;

% Necessary to set bounds dynamically
codeoptions.nlp.stack_parambounds = 1;
%codeoptions.dump_formulation = 1;

% Speed up the solver
%codeoptions.nlp.checkFunctions = 0;
%codeoptions.nlp.linear_solver = 'symm_indefinite_fast';

%% Generate forces solver
FORCES_NLP(model, codeoptions);

%% Simulation

%param_data = textread('all_params.txt', '%f', 'delimiter', '\n');
param_data = [];
for i=1:model.N
    k = track.r(i);
    param_k = [.5, .5, 240, 93, .708, .822, 3152.3, 2785.4, 1.6, 1.6, 10.1507, 10.8529, .45, 9.81, .8727, 1.255, 1, .1, 0.5, k, .25, 0, 0.4]';
    param_data = [param_data;param_k];
end
%x0 = textread('x0.txt', '%f', 'delimiter', '\n');
Dr = param_data(7);
Df = param_data(8);
lambda = param_data(23);
z_lb = [-deg2rad(40)*0.025,-10*0.025,-deg2rad(23),-10,-2,-deg2rad(40),0,-2,-2*pi]';
z_ub = [deg2rad(40)*0.025,10*0.025,deg2rad(23),10,2,deg2rad(40),30,2,2*pi]';
problem.all_parameters = param_data;
problem.lb = repmat(z_lb,model.N,1);
problem.ub = repmat(z_ub,model.N,1);
problem.hu = repmat([1.5;1.5;(Dr*lambda)^2;(Df*lambda)^2],model.N,1);
problem.hl = repmat([0.001;0.001;0.001;0.001],model.N,1);

x0 = [0,0,0,0,1,0.2,10,0.2,0]';
%x0 = zeros(9,1);
problem.x0 = repmat(x0,model.N,1);

%problem.xinit = textread('xinit.txt', '%f', 'delimiter', '\n');
xinit = zeros(9,1);
problem.xinit = xinit;
%problem.xfinal = xinit(5:6);

% Time to solve the NLP!
[output,exitflag,info] = CURVATUREsolver(problem);
exitflag

function f = objective(z,p)
    delta_d = z(1);
    delta_a = z(2);
    steering = z(1) + z(3);
    n = z(5);
    mu = z(6);
    vx = z(7);
    vy = z(8);
    w = z(9);
    
    Ts = 25e-3;
    Lf = p(5);
    Lr = p(6);
    dRa = p(1);
    dRd = p(2);
    q_slip = p(18);
    k = p(20);
    q_n = p(21);
    q_mu = p(22);
    
    % Time minimization term
    min_time = (vx*cos(mu) - vy*sin(mu))/(1 - n*k)* Ts;
    
    % Slip difference
    beta_dyn = atan(vy/vx);
    beta_kin = atan(steering*Lr/(Lr+Lf));
    diff_beta = beta_dyn - beta_kin;
    
     f = -min_time + q_n*(n)^2 + (delta_d)^2*dRd + ...
         + (delta_a)^2*dRa + q_slip*(diff_beta)^2 + q_mu*(mu)^2;

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
    
    % Differential equations
    xdot = [vx*sin(mu) + vy*cos(mu);
            (w - k*(vx*cos(mu) - vy*sin(mu))/(1 - n*k));
            (1/m)*(Fx - Ff*sin(delta) + m*vy*w);
            (1/m)*(Fr + Ff*cos(delta) - m*vx*w);
            (1/I)*(Ff*Lf*cos(delta) - Fr*Lr)];
end

function xnext = dynamics(z,p)
    u = z(1:2);
    x = z(3:9);
    U = u + x(1:2);
    % implements a RK4 integrator for the dynamics
    integrator_stepsize = 0.025;
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
    -n + length/2*sin(abs(mu)) + width/2*cos(mu);
    Fe_r; Fe_f];
end