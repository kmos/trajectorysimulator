close all
clc
fullscreen = get(0,'ScreenSize');
p = figure('Position',[0 0 fullscreen(3) fullscreen(4)]);
set(p, 'color', 'white');
[time, air] = size(x');

for i = 1:300:time
    PlotTarget

 %% Traiettoria velivo
    w = plot3(x(1:i), y(1:i), z(1:i));
    set(w,'LineWidth',1,'color','b')
    drawAircraft([y(i), x(i), -z(i)],[Phi(i), Theta(i), Psi(i)],50,'b');
    hold on

%% Traiettoria indotta
    w = plot3(xg(1:i), yg(1:i), zg(1:i));
    set(w,'LineWidth',1,'color','r')
    drawAircraft([yg(i), xg(i), -zg(i)],[Phig(i), Thetag(i), Psig(i)],50,'r');

%% Traiettoria con errori       
%     w = plot3(xErr(1:i), yErr(1:i), zErr(1:i));
%     set(w,'LineWidth',1,'color','g')
%     drawAircraft([yErr(i), xErr(i), -zErr(i)],[PhiErr(i), ThetaErr(i), PsiErr(i)],50,'g');

%%    
    hold off
    pause(0.05)
end