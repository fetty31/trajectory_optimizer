function plots(S,X_OPT)
names = {'delta d','delta a','delta','a','n','mu','Vx','Vy','w'};
for i=1:size(X_OPT,1)
    figure()
    %plot(S(1:end-1),X_OPT(i,:))
    plot(S,X_OPT(i,:))
    xlabel('S [m]')
    ylabel(names(i))
    hold off
end
end
