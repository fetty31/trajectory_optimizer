%% (script in complement with color_plots.m & heading_calculation.m) MUST EXECUTE the mentioned scripts before!
coord = [];
XS =[];
YS=[];
idx = 1;

% MoreLaps! --> change if git branch is all-in-one or more-laps
moreLaps = false;
meters = 50;

index = delta_s/0.05; % discretization of track indexes 

heading = heading(1:index:length(filtered_points.Var1)-1);

if(moreLaps)
    heading = [heading(end-meters/delta_s:end);heading(2:end)];
end

cones = readtable('Germany2019.txt');
yellow = [];
blue = [];

for i=1:size(cones,1)
    if(cones.Var3(i) == 1)
        blue = [blue;[cones.Var1(i),cones.Var2(i)]];
    else
        yellow = [yellow;[cones.Var1(i),cones.Var2(i)]];
    end
end

for e=1:index:length(filtered_points.Var1) %(gro_track.curvature)%(track.r)

%     % filtered_points from Rodas
%     XS(idx,1) = filtered_points.Var2(e);
%     YS(idx,1) = filtered_points.Var3(e);
    
    % filtered_points from GRO
    XS(idx,1) = filtered_points.Var1(e);
    YS(idx,1) = filtered_points.Var2(e);
    idx=idx+1;
end

if(moreLaps)
    XS = [XS(end-meters/delta_s:end);XS];
    YS = [YS(end-meters/delta_s:end);YS];
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

% [L,~] = fcurvature(coord(:,1:2)); % calculate curvature

%% Plots

figure()
plot(coord(:,1),coord(:,2),'LineWidth',2)
hold on
plot(XS,YS,'Color','k','LineWidth',1.5);
hold on
plot(XS(1),YS(1),'o','Color','k');

hold on
plot(yellow(:,1),yellow(:,2),'o','Color','#EDB120','MarkerSize',5)
hold on
plot(blue(:,1),blue(:,2),'o','Color','b','MarkerSize',5)
hold on

% plot(filtered_points.Var1,filtered_points.Var2)

meters = 20; % meters to add markers for refence
for marker=1:round(delta_s*N/meters)-1
    if marker == 1
        plot(coord(marker*round(meters/delta_s),1),coord(marker*round(meters/delta_s),2),'o','Color','r')
    else
        plot(coord(marker*round(meters/delta_s),1),coord(marker*round(meters/delta_s),2),'x','Color','r')
    end
end

% figure()
% plot(coord(:,1),coord(:,2),'Color','k','LineWidth',1.5)
% hold on
% plot(coord(1,1),coord(1,2),'o','Color','k')
% hold on
% plot(yellow(:,1),yellow(:,2),'o','Color','#EDB120','MarkerSize',5)
% hold on
% plot(blue(:,1),blue(:,2),'o','Color','b','MarkerSize',5)

%% Color plots

figure()
coord_vx = [coord,X_OPT(7,2:end)'];
tbl = array2table(coord_vx,'VariableNames',{'x','y','idx','vx'});
% s = scatter(tbl,'x','y','filled','ColorVariable','vx');
s = scatter(tbl.x,tbl.y,30,tbl.vx,'filled');
colorbar



