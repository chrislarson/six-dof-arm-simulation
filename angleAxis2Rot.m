% angleAxis2Rot Returns the rotation matrix encoded by an angle-axis rotation of theta radians about the unit vector k axis.
% Note: Omega = theta * k
%
% R = angleAxis2Rot(Omega)
%
% Outputs:
% R = 3x3 rotation matrix corresponding to a rotation by theta about k
%
% Inputs:
% Omega = Rotation encoded as angle-axis to be converted to a 3x3 rotation
% matrix
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function R = angleAxis2Rot(Omega)
theta = norm(Omega);
k = Omega/theta;

if theta == 0
    R = eye(3);
else
    R = cos(theta)*eye(3) + sin(theta)*cpMap(k) + (1-cos(theta))*(k*k');
end

end
