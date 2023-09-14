%% path
addpath('/home/fetty/Escritorio/LaptimeSimulator/LapTime_2021_22/CASADI');


%% import
import casadi.*

%% NLP + embeddable integrator

%formulate DAE
x = SX.sym('x',2);
z = SX.sym('z');
u = SX.sym('u');
f = [z*x(1)-x(2)+u;x(1)];
g = x(2)^2+z-1;
h = x(1)^2+x(2)^2+u^2;
dae = struct('x',x,'p',u,'ode',f,'z',z,'alg',g,'quad',h);

%solver instance
T = 10;%end time
N = 20;%discretization
op = struct('t0',0,'tf',T/N);
F = integrator('F','idas',dae,op);

%empty NLP
w={}; lbw = []; ubw = [];
G = {}; J = 0;

%initial conditions
Xk = MX.sym('X0',2);
w{end+1} = Xk;
lbw = [lbw;0;1];
ubw = [ubw;0;1];

for k=1:N
    %local control
    Uname = ['U' num2str(k-1)];
    Uk = MX.sym(Uname);
    w{end+1}=Uk;
    lbw = [lbw;-1];
    ubw = [ubw; 1];
    
    %call integrator
    Fk = F('x0',Xk,'p',Uk);
    J = J+Fk.qf;
    
    %new local state
    Xname = ['X' num2str(k)];
    Xk = MX.sym(Xname,2);
    w{end+1}=Xk;
    lbw=[lbw;-25;-inf];
    ubw=[ubw;inf;inf];
    
    %continuity constraint
    G{end+1}=Fk.xf-Xk;
end

%create NLP solver
nlp = struct('f',J,'g',vertcat(G{:}),'x',vertcat(w{:}));
S = nlpsol('S','blocksqp',nlp);

%solve NLP
r = S('lbx',lbw,'ubx',ubw,'x0',0,'lbg',0,'ubg',0);
disp(r.x);
