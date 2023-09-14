clear all
load('EnduranceSpain.mat');
gro_track = readtable('gro_0.0490_0_101.csv');

order = 5;
framelen = 601;
% x = track.r;
x = gro_track.curvature;

sgf = sgolayfilt(x,order,framelen);
% sgf_t = table(sgf);
% sgf_t.Properties.VariableNames = {'curvature'};
% writetable(sgf_t,'gro_sgf.csv')

figure()
plot(x,':')
hold on
plot(sgf)
legend('original','filtered')