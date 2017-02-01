% Plot target
for t = 1:TargetNumber
    h = plot3(x_target(t), y_target(t), TargetAltitude(t));
    set(h,'marker','o','markersize',20,'LineWidth',4,'color','r')
    hold on
    hh = text(double(x_target(t)), double(y_target(t)), double(TargetAltitude(t)), num2str(t));
    set(hh,'FontSize',20)
    
end
h = line([-AreaLenght/2 -AreaLenght/2], [-AreaWidth/2 AreaWidth/2]);
set(h,'LineWidth',2,'color','k')
h = line([AreaLenght/2 AreaLenght/2], [-AreaWidth/2 AreaWidth/2]);
set(h,'LineWidth',2,'color','k')
h = line([-AreaLenght/2 AreaLenght/2], [-AreaWidth/2 -AreaWidth/2]);
set(h,'LineWidth',2,'color','k')
h = line([-AreaLenght/2 AreaLenght/2], [AreaWidth/2 AreaWidth/2]);
set(h,'LineWidth',2,'color','k')
%title(['Target distribution - Number of targets = '  num2str(TargetNumber) ' - Minimum distance among all targets = '  num2str(EffectiveMinDistAmongTarget) 'm'])%, Simulation time = ' num2str(i*Tc) 's'])
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
axis([-AreaLenght/2-10 AreaLenght/2+10 -AreaWidth/2-10 AreaWidth/2+10])
grid on
axis equal