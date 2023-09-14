%% Quick visualization of TRO's output (optimizer.cpp)
clear all;

% N = 665; %652; %693;

problem = readtable('problem.csv');
% N = problem.Var2(1); % File from TRO.py
% T = problem.Var2(2);
N = problem.Var1(1);
delta_s = problem.Var1(2);

n_states = 7;
n_controls = 2;
x_opt = readtable('x_opt.csv');
% x_opt = x_opt.Var2; % File from TRO.py
x_opt = x_opt.Var1;

X_OPT = [0;0;x_opt(1:7)];
idu = 1;
idx = 1;
for k=1:N
    X_OPT = [X_OPT,[x_opt(n_states+N*n_states+idu:n_states+N*n_states+idu+1);x_opt(n_states+idx:2*n_states+idx-1)]];
    idu = idu+2;
    idx = idx+7;
end

S = 0:delta_s:delta_s*N;

figure()
subplot(2,2,1)
plot(S,X_OPT(7,:));
title('Vx');
xlabel('S [m]');
ylabel('Vx [m/s]');

subplot(2,2,2)
plot(S,X_OPT(3,:))
title('Delta');
xlabel('S [m]');
ylabel('delta [rad]');

subplot(2,2,3)
plot(S,X_OPT(8,:))
title('Vy');
xlabel('S [m]');
ylabel('Vy [m/s]');

subplot(2,2,4)
plot(S,X_OPT(9,:))
title('w');
xlabel('S [m]');
ylabel('w [rad/s]');
hold off
    
    