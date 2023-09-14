%% MULTIPLE SHOOTING
% clear all
close all
%% path
addpath('/home/fetty/Escritorio/LaptimeSimulator/LapTime_2021_22/CASADI');

%% import
import casadi.*

%% Load
% load('EnduranceSpain.mat')
% track.r = sgf;

% load('TrackTFGDef.mat');
% gro_track = readtable('gro2_0.0490.csv');

gro_track = readtable('gro_0.0490_0_601.csv');

%% NMPC
tic
% z = [diff_d, diff_a, delta, a, n, mu, vx, vy, w] = [u,x]
T = 0.4897; %0.5 % %discretization space [m]
N = 326; %693; % prediction horizon
% Npar = 31+length(track.r(2:N+1));
Npar = 31+length(gro_track.curvature(2:N+1)); %23 + initial state (7) + k's

n_controls = 2;
n_states = 7;

%{
delta = SX.sym('delta'); 
a = SX.sym('a'); 
n = SX.sym('n');
mu = SX.sym('mu');
vx = SX.sym('vx');
vy = SX.sym('vy');
w = SX.sym('w'); %yaw rate

P = SX.sym('P',Npar);%real time parametres 

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
%}

P = SX.sym('P',Npar);%real time parametres

dRa = P(2);
dRd = P(1);
Lf = P(5);
Lr = P(6);
q_slip = P(18);
q_n = P(21);
q_s = P(24);
q_mu = P(22);

U = SX.sym('U',n_controls,N);
X = SX.sym('X',n_states,N+1);

%{
% compute solution symbolically
X(:,1) = P(24:end); % initial state
for i = 1:N
    st = X(3:end,i);  con = U(:,i) + X(1:2,i);
    f_value  = mapping(st,con,P);
    st_next  = st + T*f_value;
    X(:,i+1) = [con;st_next];
end
%}

% function to get the optimal trajectory knowing the optimal solution
%ff=Function('ff',{U,P},{X}); %FUNCIO A CANVIAR -- P NO ES LA SOLUCIO

%---------------------------------------------------------------------------------------------------------------------------------------------------------

obj = 0; % objective function
g = [];  % constraints vector

st = X(:,1); % initial state
g = [g;st-P(25:31)];

% compute objective + next state constraints
for e=1:N
    st = X(:,e);
    k = P(31+e);
    steering = st(1)+U(1,e);
    
    % Progress rate
    sdot = (st(5)*cos(st(4)) - st(6)*sin(st(4)))/(1 - st(3)*k);

    % Slip difference
    beta_dyn = atan(st(6)/st(5));
    beta_kin = atan(steering*Lr/(Lr+Lf));
    diff_beta = beta_dyn - beta_kin;

    %Objective function
    obj = obj - q_s*sdot + dRd*(U(1,e))^2 + dRa*(U(2,e))^2 + q_slip*(diff_beta)^2 + q_mu*(st(4))^2 + q_n*(st(3))^2;
    
    st_next = X(:,e+1); %next state
    con2 = U(:,e) + X(1:2,e);
    st2 = X(3:end,e);
    %stt = [con2;X(3:end,e)];  
    f_value = mapping(st2,con2,P,k);
    %f_value = f(stt);
    st_next_euler = [con2;st2 + T.*f_value]; %next state calculated with euler method
    
    g = [g;st_next-st_next_euler]; %constraints -- difference between next states = 0
    
end

%{
st = X;
k = P(31:end);
steering = st(1,:)+U(1,:);
% Progress rate
sdot = (st(5,:)*cos(st(4,:)) - st(6,:)*sin(st(4,:)))/(1 - st(3,:)*k);

% Slip difference
beta_dyn = atan(st(6,:)/st(5,:));
beta_kin = atan(steering*Lr/(Lr+Lf));
diff_beta = beta_dyn - beta_kin;

%Objective function
obj = -sdot + (U(1,:))^2*dRd + (U(2,:))^2*dRa + q_slip*(diff_beta)^2; %CHANGE U² FOR MATRIX MULTIPLYING!!
%}


long = 2.72;
width = 1.2 + 0.3;
% compute constraints to ensure staying inside the track
for j = 1:N+1   
    g = [g ; X(3,j) + long/2*sin(abs(X(4,j))) + width/2*cos(X(4,j))];   
    g = [g ; -X(3,j) + long/2*sin(abs(X(4,j))) + width/2*cos(X(4,j))];  
    %{
    h = [n - length/2*sin(abs(mu)) + width/2*cos(mu);
        -n + length/2*sin(abs(mu)) + width/2*cos(mu)];
    %}
end

g = [g;X(3:end,2)-X(3:end,end)]; %initial stage == final stage


% make the decision variables one column vector
OPT_variables = [reshape(X,n_states*(N+1),1);reshape(U,n_controls*N,1)];
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level = 5;%0,5,12
opts.print_time = true;
%opts.ipopt.hessian_constant = 'yes';
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;
opts.ipopt.expect_infeasible_problem = 'yes';

TROsolver = nlpsol('TROsolver', 'ipopt', nlp_prob, opts); %solver constructor

args = struct;
args.lbg(1:n_states*(N+1),1) = -0.01; %[st_next-st_next_euler] boundaries
args.ubg(1:n_states*(N+1),1) = 0.01;

args.lbg(n_states*(N+1)+1:n_states*(N+1)+2*(N+1),1) = 0; %inside-track boundaries 
args.ubg(n_states*(N+1)+1:n_states*(N+1)+2*(N+1),1) = 1.5;

args.lbg(n_states*(N+1)+2*(N+1)+1:n_states*(N+1)+2*(N+1)+(n_states-2),1) = [-0.1;-.5;-0.1;-2;-5]; %initial==final boundaries
args.ubg(n_states*(N+1)+2*(N+1)+1:n_states*(N+1)+2*(N+1)+(n_states-2),1) = [0.1;.5;0.1;2;5];


z_lb = [-deg2rad(3),-5,-deg2rad(23),-10,-2,-deg2rad(100),0,-2,-18.85]'; 
z_ub = [deg2rad(3),.25,deg2rad(23),6.5,2,deg2rad(100),30,2,18.85]';


%variables boundaries
args.lbx(1:n_states*(N+1),1) = repmat(z_lb(3:end),N+1,1); % x boundaries
args.ubx(1:n_states*(N+1),1) = repmat(z_ub(3:end),N+1,1);
args.lbx(n_states*(N+1)+1:length(OPT_variables),1) = repmat(z_lb(1:2),N,1); % u boundaries
args.ubx(n_states*(N+1)+1:length(OPT_variables),1) = repmat(z_ub(1:2),N,1);


%%               SIMULATION LOOP 
%--------------------------------------------------------------------------

%x0 = zeros(1,n_states); % initial condition.
x0 = abs(z_lb +(z_ub -z_lb)/2);
u0 = x0(1:2)';
x0 = x0(3:end)';
% x0 = [0,0,0.001,0.001,15,0.001,0];
% u0 = [0,0];


%x0 = [-4.78083696084108e-05;-9.59507070023394e-14;0.00195219163039159;0.00199999999990405;0.00350676599631230;0.00236118066218264;14.9691567235588;0.000790681250219699;-9.34563656274580e-05];
%x0 = [-6.346032466946045e-04;2.606023723481642e-01;-3.995572342494976e-01;3.130192937032416e+00;-7.783306105970277e-01;1.654339659208416e-02;1.498477649909190e+00;-1.999779364359543e+00;6.282926803931951e+00];
%x0 = [-8.64734273043873e-05;-1.66787737809773e-12;0.00230464520858401;0.00339999997224533;-0.591597634982950;-0.0346111101918377;24.6032357432481;0.0130468812803833;-0.0527774301796576];


k = gro_track.curvature(1);
% k = track.r(1);
% param = [dRd, dRa, m, I, Lf, Lr, Dr, Df, Cr, Cf, Br, Bf, u_r, g, Cd, rho, Ar, q_slip, p_long, k, q_n, q_mu, lambda, q_s, x0]
param_k = [1, .3, 240, 93, .708, .822, 3152.3, 2785.4, 1.6, 1.6, 10.1507, 10.8529, .45, 9.81, .8727, 1.255, 1, .1, 0.5, k, .5, .3, 2, 1, x0]';
curvature = gro_track.curvature(1:10:length(gro_track.curvature));
% curvature = track.r(1:10:end);
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
toc


%{
tic
STATS = {};
X_OPT = [];
idx=1;
for H=1:10:length(track.r(1:200))%H=1
    k = track.r(H);
    args.p = [.5, .5, 240, 93, .708, .822, 3152.3, 2785.4, 1.6, 1.6, 10.1507, 10.8529, .45, 9.81, .8727, 1.255, 1, .1, 0.5, k, .25, 0, 2, xinit']';
    args.p = [args.p;track.r(1:N)];

    u0 = repmat(u0,1,N);
    x0 = repmat(x0,1,N+1);
    args.x0  = [reshape(x0,n_states*(N+1),1);reshape(u0,n_controls*N,1)];

    solution = TROsolver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx, 'lbg', args.lbg, 'ubg', args.ubg, 'p', args.p);
    x_opt = full(solution.x);
    X_OPT(:,idx) = x_opt;
    %U_OPT(:,idx) = x_opt(;
    
    %x0 = x_opt(8:14);
    %u0 = x_opt(15:16);
    xinit = x_opt(N*7+1:N*7+7);
    u0 = [0,0]';
    x0 = [0,0,0.001,0.001,29.9,0.001,0]';
    
    sol_stats = TROsolver.stats();
    stats = sol_stats.return_status;
    STATS{idx} = {stats};

    idx = idx+1;
end
toc
%}



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
f_values = [(vx*sin(mu) + vy*cos(mu))/sdot;
        w/sdot - k;
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

