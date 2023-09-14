% load('TrackTFGDef.mat');
load('EnduranceSpain.mat')

gro_track = readtable('gro_0.0490_0_101.csv');

filtered_points = readtable('filtered_points.csv');

% x = track.X;
% y = track.Y;
% x = gro_track.x;
% y = gro_track.y;
x = filtered_points.Var2;
y = filtered_points.Var3;
heading = zeros(length(x)-1,1);
for k=1:length(x)-1
    xv = x(k+1)-x(k);
    yv = y(k+1)-y(k);
    heading(k) = atan2(yv,xv);
end
