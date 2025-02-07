% quat2Rot Returns the rotation matrix that corresponds to the quaternion, stacked [q0;q_vec].
%
% R = quat2Rot(Q)
%
% Outputs:
% R = 3x3 rotation matrix
%
% Inputs:
% Q = the rotation quaternion to be converted to a rotation matrix
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function R = quat2Rot(Q)
% Hollerbach 3.10.2
q0 = Q(1);
qv = Q(2:4);
R = (q0^2 - dot(qv,qv))*eye(3) + 2*q0*cpMap(qv) + 2*(qv*transpose(qv));
end
