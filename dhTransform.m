% dhTransform Returns the homogenous transform corresponding to the provide DH parameters for a link.
%
% H = dhTransform(a, d, alpha, theta)
%
% Outputs:
% H = Homogeneous transformation matrix encoding the DH parameters for a
% link
%
% Inputs:
% a = displacement along x(i)
% d = displacement along z(i-1)
% alpha = rotation about x(i)
% theta = rotation about z(i-1)
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function H = dhTransform(a, d, alpha, theta)
tZ = [0; 0; d];
rZ = rotZ(theta);
tX = [a; 0; 0];
rX = rotX(alpha);


H = disp2Transform(tZ) * rot2Transform(rZ) * disp2Transform(tX) * rot2Transform(rX);
end


