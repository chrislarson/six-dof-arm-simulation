% rotZ Returns a rotation matrix describing a rotation about the Z axis (theta in radians).
%
% R = rotZ(theta)
%
% Outputs:
% R = 3x3 rotation matrix encoding a rotation of theta radians about Z
%
% Inputs:
% theta = the angle in radians to rotate about Z
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function R = rotZ(theta)
ct = cos(theta);
st = sin(theta);
R = [
    ct, -st, 0;
    st,  ct, 0;
    0,    0, 1;
    ];
end
