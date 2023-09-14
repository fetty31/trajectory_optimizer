%close all
coord = [];
XS =[];
YS=[];
idx = 1;

yellow = readtable('yellow.csv');
blue = readtable('blue.csv');

filtered_points = readtable('filtered_points.csv');

gro_track = readtable('gro_0.0490_0_101.csv');

% heading = heading(1:10:length(track.X)-1);
% heading = heading(1:5:length(gro_track.x)-1);

heading = heading(1:5:length(filtered_points.Var1));

for e=1:5:length(filtered_points.Var1) %(gro_track.curvature)%(track.r)
%     XS(idx,1) = gro_track.x(e);
%     YS(idx,1) = gro_track.y(e);
%     XS(idx,1) = track.X(e);
%     YS(idx,1) = track.Y(e);
    XS(idx,1) = filtered_points.Var2(e);
    YS(idx,1) = filtered_points.Var3(e);
    idx=idx+1;
end
for i=1:length(X_OPT(:,2:end))
    n = X_OPT(5,i);
    mu = X_OPT(6,i);
    vx = X_OPT(7,i);
    vy = X_OPT(8,i);
    w = X_OPT(9,i);
    
    x = XS(i) - n*sin(heading(i));
    y = YS(i) + n*cos(heading(i));
    
    coord(i,:) = [x,y,i];
    
    
end
[L,~] = fcurvature(coord(:,1:2));
figure()
plot(coord(:,1),coord(:,2),'.')
hold on
plot(XS,YS);
plot(XS(1),YS(1),'o','Color','r');
% hold on
% plot(blue.Var1,blue.Var2,'Color','b');
% plot(yellow.Var1,yellow.Var2,'Color','y');
title('0/601');