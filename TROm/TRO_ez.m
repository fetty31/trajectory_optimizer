clear all
close all
%% Load
load('EnduranceSpain.mat')
%gro_track = readtable('gro_0.0490.csv');

%% Simulation CURVATUREsolver 
N = 40;
npar = 23;
delta_s = 0.1;
%param_data = textread('all_params.txt', '%f', 'delimiter', '\n');

param_data = [];
for i=1:2:length(track.r) %length(gro_track.curvature)
    k = track.r(i); %gro_track.curvature(i)
    param_k = [.8, .5, 240, 93, .708, .822, 3152.3, 2785.4, 1.6, 1.6, 10.1507, 10.8529, .45, 9.81, .8727, 1.255, 1, .1, 0.5, k, .25, 0, 2]';
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

%problem.hu = repmat([1.5;1.5;(Dr*lambda)^2;(Df*lambda)^2],N,1);
%problem.hl = repmat([0.001;0.001;0.001;0.001],N,1);

problem.hu = repmat([1.5;1.5],N,1); %lower and upper boundaries for inequality constrains
problem.hl = repmat([0.001;0.001],N,1);

x0 = [0,0,0,0,1,0,10,0.2,0]';
%x0 = zeros(9,1);
problem.x0 = repmat(x0,N,1);

%problem.xinit = textread('xinit.txt', '%f', 'delimiter', '\n');
%xinit = zeros(9,1);
%problem.xinit = x0;
%problem.xfinal = [];

%OUTPUT = [];
X_OPT = []; %ALL solutions
FLAGS = []; %flags for every solution
s = 0;
S = [0]; %progress
tic

for kant=1:N:size(param_data,2)/2
    
    problem.all_parameters = reshape(param_data(:,kant:kant+N-1),npar*N,1);
    problem.x0 = repmat(x0,N,1);
    problem.xinit = x0;

    [output,exitflag,info] = CURVATUREsolver(problem);
    FLAGS = [FLAGS,exitflag];
    if exitflag == 1 || exitflag == 0
        x_opt = tf_output(output,N);
        X_OPT = [X_OPT,full(x_opt)];
        x0 = output.x40;
        %xinit = output.x40;
        s = s + delta_s*N;
        S = [S,S(end):delta_s:s];
    end
end
Sprima = S;
S = S(2:kant+N);

plot(S,X_OPT(7,:));
xlabel('S [m]');
ylabel('Vx [m/s]');
figure()
plot(track.X,track.Y);
hold on
plot(track.X(1),track.Y(1),'o','Color','r');
plot(track.X(24),track.Y(24),'x','Color','g');
hold off
toc


function x_opt = tf_output(output,N)
x_opt = [];
for k=1:N
    x = num2str(k);
    if k<10
        st = strcat('x0',x);
    else
        st = strcat('x',x);
    end
    x_opt(:,k) = getfield(output,st); 
end
end   