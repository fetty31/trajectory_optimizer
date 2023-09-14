clear all
close all
%% Load
load('EnduranceSpain.mat')
gro_track = readtable('gro_0.0490.csv');

%% Simulation CURVATUREsolver 
N = 40;
Nf = 80;
npar = 23;
delta_s = 0.1;
%param_data = textread('all_params.txt', '%f', 'delimiter', '\n');

param_data = [];
for i=1:2:length(track.r) %length(gro_track.curvature)
    k = track.r(i); %gro_track.curvature(i)
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

problem.lb = repmat(z_lb,N,1); %problem lower and upper boundaries
problem.ub = repmat(z_ub,N,1);

LB = repmat(z_lb,Nf,1);
UB = repmat(z_ub,Nf,1);
lb = LB(1:end-7,:); %lower and upper boundaries for finalCURVATURE solver
ub = UB(1:end-7,:);

%problem.hu = repmat([1.5;1.5;(Dr*lambda)^2;(Df*lambda)^2],N,1);
%problem.hl = repmat([0.001;0.001;0.001;0.001],N,1);

problem.hu = repmat([1.5;1.5],N,1); %lower and upper boundaries for inequality constrains
problem.hl = repmat([0.001;0.001],N,1);

x0 = [0,0,0,0,1,0,10,0.2,0]';
%x0 = zeros(9,1);
problem.x0 = repmat(x0,N,1);

%problem.xinit = textread('xinit.txt', '%f', 'delimiter', '\n');
xinit = zeros(9,1);
%problem.xinit = xinit;
%problem.xfinal = [];

%OUTPUT = [];
X_OPT = []; %ALL solutions
FLAGS = []; %flags for every solution
curv = [];%curvature from solutions
kant = 1; k = 1;
smax = round(track.x(end),1); %round(sum(gro_track.dist),1) %total length (progress) of the trajectory
tcontrol = 4; % solution that the mpc takes (arbitrary) 
s = 0;
S = [s]; %progress
tic

fprintf('%s\n','CURVATUREsolver working...')

while s<smax
       
    if abs(s-smax)>1
        problem.all_parameters = reshape(param_data(:,kant:kant+N-1),npar*N,1);
        problem.x0 = repmat(x0,N,1);
        
        [output,exitflag,info] = CURVATUREsolver(problem);
        %OUTPUT = [OUTPUT;full(output)];
        FLAGS(k) = exitflag;
        if exitflag == 1
            X_OPT = [X_OPT,full(output.x04)];
            curv = [curv,param_data(20,kant+tcontrol)];
            x0 = output.x04;
            if s==0
                x0init = x0;
            end
            %sdot = (x0(7)*cos(x0(6)) - x0(8)*sin(x0(6)))/(1 - x0(5)*problem.all_parameters(npar*tcontrol-3));
            s = s + delta_s*tcontrol;
            S = [S,s];
        end
    else
        fprintf('%s\n','finalCURVATUREsolver working...Good luck')
        problem.all_parameters = reshape(param_data(:,kant:kant+Nf-1),npar*Nf,1);
        problem.x0 = repmat(x0,Nf,1);
        %problem.xinit = X_OPT(1:9,end);
        problem.xfinal = x0init(3:end);
        problem.lb = lb;
        problem.ub = ub;
        problem.hu = repmat([1.5;1.5],Nf,1); %lower and upper boundaries for inequality constrains
        problem.hl = repmat([0.001;0.001],Nf,1);
        %curvature = kant;
        [output,exitflag,info] = finalCURVATUREsolver(problem);
        %OUTPUT = [OUTPUT;full(output)];
        FLAGS(k) = exitflag;
        if exitflag == 1
            X_OPT = [X_OPT,full(output.x04)];
            curv = [curv,param_data(20,kant+tcontrol)];
            x0 = output.x04;
            %sdot = (x0(7)*cos(x0(6)) - x0(8)*sin(x0(6)))/(1 - x0(5)*problem.all_parameters(npar*tcontrol-3));
            s = s + delta_s*tcontrol;
            S = [S,s];
        end
    end
       
    kant = kant+tcontrol;
    k = k+1;
end
plot(S(1:end-1),X_OPT(7,:));
xlabel('S [m]');
ylabel('Vx [m/s]');
hold on
figure()
%plot(track.X,track.Y);
plot(gro_track.x,gro_track.y);
hold on
%plot(track.X(1),track.Y(1),'o','Color','r');
%plot(track.X(24),track.Y(24),'o','Color','g');
plot(gro_track.x(1),gro_track.y(1),'o','Color','r');
plot(gro_track.x(24),gro_track.y(24),'x','Color','g');
hold off
%{
kant = 1;
for k=1:693/33
    problem.all_parameters = repmat(param_data(kant:k*npar,1),N,1);
    % Time to solve the NLP!
    [output,exitflag,info] = CURVATUREsolver(problem);
    FLAGS(k) = exitflag;
    
    kant = k*npar+1;
end
%}
toc