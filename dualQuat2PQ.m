% dualQuat2PQ Returns a position and a quaternion that represent the transform encoded in the dual quaternion structure, which has members rot and disp.
% rot is a 4x1 unit quaternion stacked [q0;q_vec].
% disp is a 4x1 quaternion stacked [q0;q_vec] that encodes the dual rotation-mixed translation.
%
% [pos, quat] = dualQuat2PQ(dual_quat)
%
% Outputs:
% pos = the 3d position
% quat = the quaternion encoding rotation
%
% Inputs:
% dual_quat = dual quaternion to be converted to position and rotation
% quaternion
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function [pos, quat] = dualQuat2PQ(dual_quat)
quat = dual_quat.rot;
quat_conj = [quat(1); -quat(2); -quat(3); -quat(4)];
pos_q = 2 * multiplyQuat(dual_quat.disp, quat_conj);
pos = [pos_q(2); pos_q(3); pos_q(4)];
end
