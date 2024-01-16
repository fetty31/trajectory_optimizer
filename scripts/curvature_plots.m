%% Visualization of curvature used as input in TRO's solver (optimization.cpp) --> MUST execute color_plots.m before
filtered_curvature = readtable('../data/filtered_curvature.csv');
curvature = readtable('../data/curvature.csv');

filtered_curvature = filtered_curvature.Var1; % Filtered curvature is already in TRO discretization
curvature = curvature.Var1;

curvature = curvature(1:delta_s/0.05:end); % Set curvature with TRO discretization
curvature = curvature(2:end);

progres = 0:delta_s:delta_s*N;

figure()
plot(curvature)
hold on
plot(filtered_curvature)
title('Curvature comparison')
