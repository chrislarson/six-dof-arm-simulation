% twist2Transform Returns the homogenous transformation matrix corresponding to a 6 element twist vector. The twist should be stacked [v;w th].
%
% H = twist2Transform(t)
%
% Outputs:
% H = homogeneous transformation matrix encoding the supplied twist vector
% transformation
%
% Inputs:
% t = twist vector encoding the transformation to be converted into a
% homogeneous transformation matrix
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function H = twist2Transform(t)
v = t(1:3);
Omega = t(4:6);

d = v;
R = angleAxis2Rot(Omega);

theta = norm(Omega);
if theta ~= 0
    k = Omega/theta;
    d = ((eye(3)-R)*cpMap(k) + theta*(k*k'))*v;
end

H = [
    R, d;
    0, 0, 0, 1;
    ];
end
