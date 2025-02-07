% rotX Returns a rotation matrix describing a rotation about the X axis (theta in radians).
%
% R = rotX(theta)
%
% Outputs:
% R = 3x3 rotation matrix encoding a rotation of theta radians about X
%
% Inputs:
% theta = the angle in radians to rotate about X
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function R = rotX(theta)
ct = cos(theta);
st = sin(theta);
R = [
    1,  0,   0;
    0, ct, -st;
    0, st,  ct;
    ];
end
