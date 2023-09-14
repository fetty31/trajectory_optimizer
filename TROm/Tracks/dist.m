
track1 = readtable('gro_0.0490_0_101.csv');
track2 = readtable('gro_0.0490_0_301.csv');
track3 = readtable('gro_0.0490_0_501.csv');
track4 = readtable('gro_0.0490_0_601.csv');
track5 = readtable('gro_0.0490_0_701.csv');
track6 = readtable('gro_0.0490_0_1001.csv');

d1 = [0];
d2 = d1;
d3 = d1;
d4 = d1;
d5 = d1;
d6 = d1;
for e=2:length(track1.curvature)-1
    d1(e,1) = d1(e-1,1)+track1.dist(e);
    d2(e,1) = d2(e-1,1)+track2.dist(e);
    d3(e,1) = d3(e-1,1)+track3.dist(e);
    d4(e,1) = d4(e-1,1)+track4.dist(e);
    d5(e,1) = d5(e-1,1)+track5.dist(e);
    d6(e,1) = d6(e-1,1)+track6.dist(e);
end

figure()
subplot(2,3,1)
plot(d1,track1.curvature(1:end-1))
title('0/101') 
subplot(2,3,2)
plot(d2,track2.curvature(1:end-1))
title('0/301') 
subplot(2,3,3)
plot(d3,track3.curvature(1:end-1))
title('0/501') 
subplot(2,3,4)
plot(d4,track4.curvature(1:end-1))
title('0/601') 
subplot(2,3,5)
plot(d5,track5.curvature(1:end-1))
title('0/701') 
subplot(2,3,6)
plot(d6,track6.curvature(1:end-1))
title('0/1001') 


% gro_track = readtable('gro_0.0490_1.csv');
d = [0];
for e=2:length(gro_track.curvature)-1
    d(e,1) = d(e-1,1)+gro_track.dist(e);
end

% 
% figure()
% subplot(1,2,1)
% plot(d,gro_track.curvature(1:end-1))
% 
% [~,R] = fcurvature([gro_track.x,gro_track.y]);
% k = 1./R;
% k = k(2:end-1);
% 
% subplot(1,2,2)
% plot(d(2:end),k)