clear all
close all
%% path
addpath('/home/fetty/Escritorio/LaptimeSimulator/LapTime_2021_22/CASADI');

%% import
import casadi.*

%% SOLVER
N = 40; % horizon length
Npar = 23; % nº real time parameters

%% Variables
delta = SX.sym('delta'); 
a = SX.sym('a'); 
n = SX.sym('n');
mu = SX.sym('mu');
vx = SX.sym('vx');
vy = SX.sym('vy');
w = SX.sym('w'); %yaw rate
x = [delta; a; n; mu; vx; vy; w];

delta_d = SX.sym('diff_d'); 
delta_a = SX.sym('diff_a');
u = [delta_d; delta_a];

P = SX.sym('P',Npar);%real time parameters 

%% Model equations
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

sdot = (vx*cos(mu) - vy*sin(mu))/(1 - n*k);% Progress rate minimization term

xdot = [vx*sin(mu) + vy*cos(mu)/sdot;
           (w - k*(vx*cos(mu) - vy*sin(mu))/(1 - n*k))/sdot;
           (1/m)*(Fx - Ff*sin(delta) + m*vy*w)/sdot;
           (1/m)*(Fr + Ff*cos(delta) - m*vx*w)/sdot;
           (1/I)*(Ff*Lf*cos(delta) - Fr*Lr)/sdot];

%% Objective term
steering = u(1)+x(1);
beta_dyn = atan(vy/vx); 
beta_kin = atan(steering*Lr/(Lr+Lf));
diff_beta = beta_dyn - beta_kin; % Slip difference

obj = -sdot + (delta_d)^2*dRd + (delta_a)^2*dRa + q_slip*(diff_beta)^2;

%% Formulate discrete time dynamics

% Fixed step Runge-Kutta 4 integrator
h = 0.2; %sampling   
f = Function('f', {x, u}, {xdot, obj});
X0 = MX.sym('X0', length(x));
U = MX.sym('U',length(u));
X = X0;
Q = 0;

st = X(3:end,k);  con = U(:,k) + X(1:2,k);
f_value  = mapping(st,con,P);
st_next  = st + T*f_value;
X(:,k+1) = [con;st_next];
    
k1 = f(X, U);
k2 = f(X + h/2 * k1, U);
k3 = f(X + h/2 * k2, U);
k4 = f(X + h * k3, U);

X=X+h/6*(k1 +2*k2 +2*k3 +k4);


F = Function('F', {X0, U}, {X, Q}, {'x0','u_'}, {'xf', 'qf'});


% Evaluate at a test point
Fk = F('x0',[0,0,0,0,1,0.2,10,0.2,0],'u_',0.4);
disp(Fk.xf)
disp(Fk.qf)

% Start with an empty NLP
w={};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g={};
lbg = [];
ubg = [];

% Formulate the NLP
Xk = [0; 1];
for k=0:N-1
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)]);
    w = {w{:}, Uk};
    lbw = [lbw, -1];
    ubw = [ubw,  1];
    w0 = [w0,  0];

    % Integrate till the end of the interval
    Fk = F('x0',Xk,'p', Uk);
    Xk = Fk.xf;
    J=J+Fk.qf;

    % Add inequality constraint
    g = {g{:}, Xk(1)};
    lbg = [lbg; -.25];
    ubg = [ubg;  inf];
end

% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
solver = nlpsol('solver', 'ipopt', prob);

% Solve the NLP
sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw, ...
             'lbg', lbg, 'ubg', ubg);
w_opt = full(sol.x);

% Plot the solution
u_opt = w_opt;
x_opt = [0;1];
for k=0:N-1
    Fk = F('x0', x_opt(:,end), 'p', u_opt(k+1));
    x_opt = [x_opt, full(Fk.xf)];
end
x1_opt = x_opt(1,:);
x2_opt = x_opt(2,:);
tgrid = linspace(0, T, N+1);
clf;
hold on
plot(tgrid, x1_opt, '--')
plot(tgrid, x2_opt, '-')
stairs(tgrid, [u_opt; nan], '-.')
xlabel('t')
legend('x1','x2','u')

function xdot = mapping(states,controls,P) % nonlinear mapping function f(x,u)

n = states(3);
mu = states(4);
vx = states(5);
vy = states(6);
w = states(7);

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
k = P(20);

% Slip angles
alpha_R = atan((vy-Lr*w)/(vx));
alpha_F = atan((vy+Lf*w)/(vx)) - delta;

% Pacejka lateral force magic formula
Fr = Dr*sin(Cr*atan(Br*alpha_R));
Ff = Df*sin(Cf*atan(Bf*alpha_F));
Fx = m*a - m*u_r*g - 0.5*Cd*rho*Ar*vx^2;

xdot = [vx*sin(mu) + vy*cos(mu)/sdot;
           (w - k*(vx*cos(mu) - vy*sin(mu))/(1 - n*k))/sdot;
           (1/m)*(Fx - Ff*sin(delta) + m*vy*w)/sdot;
           (1/m)*(Fr + Ff*cos(delta) - m*vx*w)/sdot;
           (1/I)*(Ff*Lf*cos(delta) - Fr*Lr)/sdot];
end
