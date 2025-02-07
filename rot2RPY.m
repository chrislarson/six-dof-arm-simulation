% rot2RPY Returns the roll, pitch and yaw corresponding to a given rotation matrix.
%
% [roll, pitch, yaw] = rot2RPY(R)
%
% Outputs:
% roll = angle of rotation about X in radians
% pitch = angle of rotation about Y in radians
% yaw = angle of rotation about Z in radians
%
% Inputs:
% R = 3x3 rotation matrix to be converted to roll, pitch, and yaw angles
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function [roll, pitch, yaw] = rot2RPY(R)
pp = atan2(-R(3,1),sqrt(R(1,1)^2 + R(2,1)^2));
pn = atan2(-R(3,1),-sqrt(R(1,1)^2 + R(2,1)^2));
yp = 0;
yn = 0;

switch pp
    case pi/2
        rp = atan2(R(1,2), R(2,2));
        rn = rp;
    case -pi/2
        rp = atan2(-R(1,2), R(2,2));
        rn = rp;
    otherwise
        rp = atan2(R(3,2)/cos(pp),R(3,3)/cos(pp));
        rn = atan2(R(3,2)/cos(pn),R(3,3)/cos(pn));
        yp = atan2(R(2,1)/cos(pp),R(1,1)/cos(pp));
        yn = atan2(R(2,1)/cos(pn),R(1,1)/cos(pn));
end

roll = [rp; rn];
pitch = [pp; pn];
yaw = [yp; yn];
end
