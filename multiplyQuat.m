% multiplyQuat Returns the quaternion product between two quaternions that are stored in 4x1 vectors as [q0;q_vec].
%
% Q = multiplyQuat(Q_left, Q_right)
%
% Outputs:
% Q = the quaternion representing the product between the two input
% quaternions
%
% Inputs:
% Q_left = quaternion to be multiplied
% Q_right = quaternion to be multiplied
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function Q = multiplyQuat(Q_left, Q_right)
% Hollerbach 3.10
ql0 = Q_left(1);
qlv = Q_left(2:4);
qr0 = Q_right(1);
qrv = Q_right(2:4);

q0 = ql0*qr0 - dot(qlv,qrv);
qv = ql0*qrv + qr0*qlv + cross(qlv,qrv);
Q = transpose([q0, qv(1), qv(2), qv(3)]);
end
