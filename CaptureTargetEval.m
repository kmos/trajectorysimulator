function [NextTarget] = CaptureTargetEval(NextTarget, TargetNumber, TargetEast, TargetNorth, TargetUp, CaptureTargetThreshold)
%#codegen

if sqrt( (TargetEast(NextTarget))^2 + (TargetNorth(NextTarget))^2 + (TargetUp(NextTarget))^2 ) - CaptureTargetThreshold < 0
    if NextTarget < TargetNumber
        NextTarget = NextTarget + 1;
    else
        NextTarget = 1;
    end
end
    


