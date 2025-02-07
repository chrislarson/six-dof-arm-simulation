% pq2DualQuat Returns a dual quaternion stored as a structure with members rot and disp.
% rot is a 4x1 unit quaternion staked [q0;q_vec].
% disp is a 4x1 quaternion stacked [q0;q_vec] that encodes the dual rotation-mixed translation.
%
% dual_quat = pq2DualQuat(pos, quat)
%
% Outputs:
% dual_quat = the dual quaternion containing rotation (rot) and
% displacement (disp) properties
%
% Inputs:
% pos = the position
% quat = the quaternion encoding rotation
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function dual_quat = pq2DualQuat(pos, quat)

quat = quat / norm(quat);
Qr = quat;
Qd = 0.5 * multiplyQuat([0;pos(1);pos(2);pos(3)], quat);

dual_quat.rot = Qr;
dual_quat.disp = Qd;


end
