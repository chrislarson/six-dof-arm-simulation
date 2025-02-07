% rotY Returns a rotation matrix describing a rotation about the Y axis (theta in radians).
%
% R = rotY(theta)
%
% Outputs:
% R = 3x3 rotation matrix encoding a rotation of theta radians about Y
%
% Inputs:
% theta = the angle in radians to rotate about Y
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function R = rotY(theta)
ct = cos(theta);
st = sin(theta);
R = [
    ct,  0, st;
    0,  1,  0;
    -st,  0, ct;
    ];
end
