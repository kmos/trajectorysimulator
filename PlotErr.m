close all
clc
fullscreen = get(0,'ScreenSize');
p = figure('Position',[0 0 fullscreen(3) fullscreen(4)]);
set(p, 'color', 'white');
%[time, air] = size(x');

indChange = find(NextTargetVet >= TargetNumber - 1);
points = [];

PLOTEZ      = 0;
PLOTEHORIZ  = 0;
PLOTE       = 1;

for i = 1:30:time
    
    
    if i >= indChange(1)
        disp(num2str(i));
        if PLOTE == 1
            %subplot(311)
            if PLOTEZ == 1 || PLOTEHORIZ == 1 
                subplot(311)
            end
            plot(E(indChange(1):i), Ev(indChange(1):i),'.b');
            hold on
            plot(E(1:indChange(1)-1), Ev(1:indChange(1)-1),'.r');
            points(i,:) = [E(i), Ev(i), 1];
        end
        
        if PLOTEZ == 1
            if PLOTE == 1 || PLOTEHORIZ == 1 
                subplot(312)
            end
            plot(ez(indChange(1):i), evz(indChange(1):i),'.b');
            hold on
            plot(ez(1:indChange(1)-1), evz(1:indChange(1)-1),'.r');
            points(i,:) = [E(i), Ev(i), 1];
        end
        
        if PLOTEHORIZ == 1
            %subplot(313)
            if PLOTE == 1 || PLOTEZ == 1 
                subplot(313)
            end
            plot(ehoriz(indChange(1):i), evhoriz(indChange(1):i),'.b');
            hold on
            plot(ehoriz(1:indChange(1)-1), evhoriz(1:indChange(1)-1),'.r');
            points(i,:) = [E(i), Ev(i), 1];
        end
        
    else
        if PLOTE == 1
            if PLOTEZ == 1 || PLOTEHORIZ == 1 
                subplot(311)
            end
            plot(E(1:i), Ev(1:i),'.r');
            points(i,:) = [E(i), Ev(i), 0];
        end
        
        if PLOTEZ == 1
            if PLOTE == 1 || PLOTEHORIZ == 1 
                subplot(312)
            end
            plot(ez(1:i), evz(1:i),'.r');
            points(i,:) = [ez(i), evz(i), 0];
        end
            
        if PLOTEHORIZ == 1
            if PLOTE == 1 || PLOTEZ == 1 
                subplot(313)
            end
            plot(ehoriz(1:i), evhoriz(1:i),'.r');
            points(i,:) = [ehoriz(i), evhoriz(i), 0];
        end 
    end
    title('Error')
    xlabel('Position error [m]')
    ylabel('Velocity error [m/s]')
    grid on
%    axis([0 70 0 6])
    hold off
    pause(0.02)
end
