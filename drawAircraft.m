function drawAircraft(ned,eul,sc,col)
%#codegen

Phi     = eul(1);
Theta   = eul(2);
Psi     = eul(3);

ala =  [ 0.2      0.2      -0.2       -0.2
        -1.5      1.5       1.5       -1.5
         0.0      0.0       0.0        0.0]*sc;       

fusx = [ 0.4      0.4      -1.5       -1.5
        -0.2      0.2       0.2       -0.2
         0.0      0.0       0.0        0.0]*sc;

fusz = [ 0.4      0.4      -1.5       -1.5
         0.0      0.0       0.0        0.0
         0.0      0.2       0.2        0.0]*sc;
     
vrt =  [-1.2     -1.2      -1.5       -1.5
         0.0      0.0       0.0        0.0
         0.0     -0.5      -0.5        0.0]*sc;
        
xbody   = [100 0 0]';
ybody   = [0 100 0]';
zbody   = [0 0 100]';

TransMat1   =   [0           1          0
                 1           0          0
                 0           0         -1];
         
         
TransMat    =   [1           0          0
                 0           cos(Phi)   sin(Phi)
                 0          -sin(Phi)   cos(Phi)]  ...
            * ...
                [cos(Theta)  0         -sin(Theta)
                 0           1          0
                 sin(Theta)  0          cos(Theta)] ...
            * ...
                [cos(Psi)    sin(Psi)   0
                -sin(Psi)    cos(Psi)   0
                 0           0          1];
    
TransMat    =   TransMat1*TransMat';
         
for i = 1:length(ala(1,:))
    ala(:,i)        = TransMat*ala(:,i);
    fusx(:,i)       = TransMat*fusx(:,i); 
    fusz(:,i)       = TransMat*fusz(:,i);
    vrt(:,i)        = TransMat*vrt(:,i);
end

xbody = TransMat*xbody;
ybody = TransMat*ybody;
zbody = TransMat*zbody;

% Draw aircraft
fill3(ned(2)+ala(1,:),  ned(1)+ala(2,:),  -ned(3)+ala(3,:),  col, ...
      ned(2)+fusx(1,:), ned(1)+fusx(2,:), -ned(3)+fusx(3,:), col, ...
      ned(2)+fusz(1,:), ned(1)+fusz(2,:), -ned(3)+fusz(3,:), col, ...
      ned(2)+vrt(1,:),  ned(1)+vrt(2,:),  -ned(3)+vrt(3,:),  col);

% Draw Body-Axes
hlx = line([ned(2) ned(2)+xbody(1)], [ned(1) ned(1)+xbody(2)], [-ned(3) -ned(3)+xbody(3)]);
set(hlx,'LineWidth',1,'Color','m');
hly = line([ned(2) ned(2)+ybody(1)], [ned(1) ned(1)+ybody(2)], [-ned(3) -ned(3)+ybody(3)]);
set(hly,'LineWidth',1,'Color','m');
hlz = line([ned(2) ned(2)+zbody(1)], [ned(1) ned(1)+zbody(2)], [-ned(3) -ned(3)+zbody(3)]);
set(hlz,'LineWidth',1,'Color','m');