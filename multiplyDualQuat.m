% multiplyDualQuat Returns the dual quaternion product between two dual quaternions that are stored in the dual quaternion structure, which has members rot and disp, and acts as a chained transformation.
%
% dual_quat = multiplyDualQuat(dual_quat_left, dual_quat_right)
%
% Outputs:
% dual_quat = the dual quaternion product of the input dual quaternions
%
% Inputs:
% dual_quat_left = dual quaternion to be multiplied
% dual_quat_right = dual quaternion to be multiplied
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function dual_quat = multiplyDualQuat(dual_quat_left, dual_quat_right)
qr1 = dual_quat_left.rot;
qd1 = dual_quat_left.disp;
qr2 = dual_quat_right.rot;
qd2 = dual_quat_right.disp;

dual_quat.rot = multiplyQuat(qr1,qr2);
dual_quat.disp = multiplyQuat(qr1, qd2) + multiplyQuat(qd1, qr2);
end
