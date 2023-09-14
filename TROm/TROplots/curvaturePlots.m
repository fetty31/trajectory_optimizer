%% Visualization of curvature used as input in TRO's solver (optimization.cpp) --> MUST execute color_plots.m before
filtered_curvature = readtable('filtered_curvature.csv');
curvature = readtable('curvature.csv');

filtered_curvature = filtered_curvature.Var1; % Filtered curvature is already in TRO discretization
curvature = curvature.Var1;

curvature = curvature(1:delta_s/0.05:end); % Set curvature with TRO discretization
curvature = curvature(2:end);

progres = 0:delta_s:delta_s*N;

figure()
subplot(1,2,1)
plot(curvature)
title('Curvature')

subplot(1,2,2)
plot(filtered_curvature)
title('Filtered Curvature')