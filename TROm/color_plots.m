%{
figure()
subplot(1,2,[1,2])
title('Prova')
x = track.X';
y = track.Y';
sz = 10;
%c = X_OPT(7,:)';
c = linspace(1,10,length(x));
scatter(x,y,sz,c,'filled')
colorbar
%}
% track05 = readtable('gro_0.0490_0.5.csv');
% track075 = readtable('gro_0.0490_0.75.csv');
% track1 = readtable('gro_0.0490_1.csv');
% track15 = readtable('gro_0.0490_1.5.csv');
% track2 = readtable('gro_0.0490_2.csv');
% track3 = readtable('gro_0.0490_3.csv');
% track4 = readtable('gro_0.0490_4.csv');
% track5 = readtable('gro_0.0490_5.csv');
% 
% delta_s = 0.4897;
% N = 328;
% 
% curvature05 = track05.curvature(1:10:length(track05.curvature));
% curvature075 = track075.curvature(1:10:length(track075.curvature));
% curvature1 = track05.curvature(1:10:length(track1.curvature));
% curvature15 = track15.curvature(1:10:length(track15.curvature));
% curvature2 = track2.curvature(1:10:length(track2.curvature));
% curvature3 = track3.curvature(1:10:length(track3.curvature));
% curvature4 = track4.curvature(1:10:length(track4.curvature));
% curvature5 = track5.curvature(1:10:length(track5.curvature));
% 
% S = 0:delta_s:delta_s*N;

% figure()
% subplot(2,4,1)
% plot(S,curvature05(1:end-2));
% title(0.5)
% 
% subplot(2,4,2)
% plot(S,curvature075(1:end-1))
% title(0.75)
% 
% subplot(2,4,3)
% plot(S,curvature1(1:end-1))
% title(1)
% 
% subplot(2,4,4)
% plot(S,curvature15)
% title(1.5)
% 
% subplot(2,4,5)
% plot(S,curvature2)
% title(2)
% 
% subplot(2,4,6)
% plot(S(1:end-1),curvature3)
% title(3)
% 
% subplot(2,4,7)
% plot(S(1:end-2),curvature4)
% title(4)
% 
% subplot(2,4,8)
% plot(S(1:end-3),curvature5)
% title(5)

% filtered_points = readtable('filtered_points.csv');
% filtered_points = filtered_points(2:end,:);
% x = filtered_points.Var2;
% y = filtered_points.Var3;
% plot(x,y)
% hold on
% plot(XS,YS)

% N = 665; %652; %693;

problem = readtable('problem.csv');
N = problem.Var2(1);

n_states = 7;
n_controls = 2;
T = 0.25; %0.4897;
x_opt = readtable('x_opt.csv');
x_opt = x_opt.Var2;

X_OPT = [0;0;x_opt(1:7)];
idu = 1;
idx = 1;
for k=1:N
    X_OPT = [X_OPT,[x_opt(n_states+N*n_states+idu:n_states+N*n_states+idu+1);x_opt(n_states+idx:2*n_states+idx-1)]];
    idu = idu+2;
    idx = idx+7;
end

delta_s = T;
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
plot(S,X_OPT(6,:))
title('mu');
xlabel('S [m]');
ylabel('mu [rad]');

subplot(2,2,4)
plot(S,X_OPT(5,:))
title('n');
xlabel('S [m]');
ylabel('n [m]');
hold off
    
    