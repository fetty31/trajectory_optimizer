%% SINGLE SHOOTING
clear all
close all

addpath('/home/fetty/Escritorio/LaptimeSimulator/LapTime_2021_22/CASADI');
import casadi.*

gro_track = readtable('gro3_0.0490.csv');

%% NMPC
% z = [diff_d, diff_a, delta, a, n, mu, vx, vy, w] = [u,x]
T = 0.5; % sampling space [m]
N = 10; % prediction horizon 328
Npar = 31+length(track.r(2:N+1)); %23 + initial state (7) + k's

delta = SX.sym('delta'); 
a = SX.sym('a'); 
n = SX.sym('n');
mu = SX.sym('mu');
vx = SX.sym('vx');
vy = SX.sym('vy');
w = SX.sym('w'); %yaw rate

P = SX.sym('P',Npar);%real time parametres 
%param = [dRd, dRa, m, I, Lf, Lr, Dr, Df, Cr, Cf, Br, Bf, u_r, g, Cd, rho, Ar, q_slip, p_long, k, q_n, q_mu, lambda, q_s]

states = [delta;a;n;mu;vx;vy;w]; 
n_states = length(states);

diff_d = SX.sym('diff_d');
diff_a = SX.sym('diff_a');

controls = [diff_d;diff_a]; 
n_controls = length(controls);

dRa = P(1);
dRd = P(2);
m = P(3);
I = P(4);
Lf = P(5);
Lr = P(6);
Dr = P(7);
Df = P(8);
Cr = P(9);
Cf = P(10);
Br = P(11);
Bf = P(12); 
u_r = P(13);
g = P(14);
Cd = P(15);
rho = P(16);
Ar = P(17);
q_slip = P(18);
p_long = P(19);
k = P(20);
q_n = P(21);
q_mu = P(22);
q_s = P(24);

% Slip angles
alpha_R = atan((vy-Lr*w)/(vx));
alpha_F = atan((vy+Lf*w)/(vx)) - delta;

% Pacejka lateral force magic formula
Fr = Dr*sin(Cr*atan(Br*alpha_R));
Ff = Df*sin(Cf*atan(Bf*alpha_F));
Fx = m*a - m*u_r*g - 0.5*Cd*rho*Ar*vx^2;

sdot = (vx*cos(mu) - vy*sin(mu))/(1 - n*k); % Progress rate minimization term

f_values = [vx*sin(mu) + vy*cos(mu)/sdot;
            w/sdot - k;
            (1/m)*(Fx - Ff*sin(delta) + m*vy*w)/sdot;
            (1/m)*(Fr + Ff*cos(delta) - m*vx*w)/sdot;
            (1/I)*(Ff*Lf*cos(delta) - Fr*Lr)/sdot];
        
f = Function('f',{states},{f_values});


U = SX.sym('U',n_controls,N);
X = SX.sym('X',n_states,N+1);

% compute solution symbolically
X(:,1) = P(25:31); % initial state
for i = 1:N
    
    k = P(31+i);
    st = X(:,e);
    steering = st(1)+U(1,e);
    
    % Progress rate
    sdot = (st(5)*cos(st(4)) - st(6)*sin(st(4)))/(1 - st(3)*k);

    % Slip difference
    beta_dyn = atan(st(6)/st(5));
    beta_kin = atan(steering*Lr/(Lr+Lf));
    diff_beta = beta_dyn - beta_kin;

    %Objective function
    obj = obj - q_s*sdot + dRd*(U(1,e))^2 + dRa*(U(2,e))^2 + q_slip*(diff_beta)^2; %+ q_mu*(st(4))^2; %+ q_n*(st(3))^2;
    
    con2 = U(:,i) + X(1:2,i);
    st2 = X(3:end,i);
    f_value = mapping(st2,con2,P,k);
    st_next = st2 + T.*f_value; %next state calculated with euler method
    
    X(:,i+1) = st_next;
end

obj = 0; % Objective function
g = [];  % constraints vector


long = 2.72;
width = 1.2 + 0.4;
% compute constraints to ensure staying inside the track
for j = 1:N+1   
    g = [g ; X(3,j) - long/2*sin(abs(X(4,j))) + width/2*cos(X(4,j))];   
    g = [g ; -X(3,j) + long/2*sin(abs(X(4,j))) + width/2*cos(X(4,j))];   
    %{
    h = [n - length/2*sin(abs(mu)) + width/2*cos(mu);
        -n + length/2*sin(abs(mu)) + width/2*cos(mu)];
    %}
end

% make the decision variables one column vector
OPT_variables = reshape(U,n_controls*N,1);
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 1000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);


args = struct;
% inequality constraints (state constraints)
args.lbg = 0.0001; %inside-track boundaries
args.ubg = 1.5;

% input constraints
args.lbx(1:2:2*N-1,1) = -deg2rad(3); args.lbx(2:2:2*N,1) = -5;
args.ubx(1:2:2*N-1,1) = deg2rad(3); args.ubx(2:2:2*N,1) = .25;


%% SIMULATION LOOP 

t0 = 0;
x0 = [0 ; 0 ; 0.0];    % initial condition.
xs = [1.5 ; 1.5 ; 0]; % Reference posture.

xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,2);  % two control inputs 

sim_tim = 20; % Maximum simulation time

% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];

k = gro_track.curvature(1);
%param = [dRd, dRa, m, I, Lf, Lr, Dr, Df, Cr, Cf, Br, Bf, u_r, g, Cd, rho, Ar, q_slip, p_long, k, q_n, q_mu, lambda, q_s, x0]
param_k = [.5, .5, 240, 93, .708, .822, 3152.3, 2785.4, 1.6, 1.6, 10.1507, 10.8529, .45, 9.81, .8727, 1.255, 1, .1, 0.5, k, .25, 0.2, 2, 2, x0]';
curvature = gro_track.curvature(1:10:length(gro_track.curvature));
curvature = curvature(1:N);
args.p = [param_k;curvature];
%args.p = param_k;
u0 = repmat(u0,1,N);
x0 = repmat(x0,1,N+1);
args.x0  = [reshape(x0,n_states*(N+1),1);reshape(u0,n_controls*N,1)];

solution = TROsolver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx, 'lbg', args.lbg, 'ubg', args.ubg, 'p', args.p);

x_opt = full(solution.x);

sol = TROsolver.stats();
stats = sol.return_status

function f_values = mapping(states,controls,P,k) % nonlinear mapping function f(x,u)

n = states(1);
mu = states(2);
vx = states(3);
vy = states(4);
w = states(5);

delta = controls(1);
a = controls(2);

m = P(3);
I = P(4);
Lf = P(5);
Lr = P(6);
Dr = P(7);
Df = P(8);
Cr = P(9);
Cf = P(10);
Br = P(11);
Bf = P(12); 
u_r = P(13);
g = P(14);
Cd = P(15);
rho = P(16);
Ar = P(17);

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
f_values = [vx*sin(mu) + vy*cos(mu)/sdot;
        (w - k*(vx*cos(mu) - vy*sin(mu))/(1 - n*k))/sdot;
        (1/m)*(Fx - Ff*sin(delta) + m*vy*w)/sdot;
        (1/m)*(Fr + Ff*cos(delta) - m*vx*w)/sdot;
        (1/I)*(Ff*Lf*cos(delta) - Fr*Lr)/sdot];

%{
f_values = [vx*sin(mu) + vy*cos(mu);
            (w - k*(vx*cos(mu) - vy*sin(mu))/(1 - n*k));
            (1/m)*(Fx - Ff*sin(delta) + m*vy*w);
            (1/m)*(Fr + Ff*cos(delta) - m*vx*w);
            (1/I)*(Ff*Lf*cos(delta) - Fr*Lr)];
%}
end

