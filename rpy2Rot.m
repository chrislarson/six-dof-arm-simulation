% rpy2Rot Returns a rotation matrix corresponding to a roll, pitch, yaw encoded rotation.
% Note: RPY is defined as the set of orthogonal rotations rotZ(yaw)rotY(pitch)rotX(roll).
%
% R = rpy2Rot(roll, pitch, yaw)
%
% Outputs:
% R = 3x3 rotation matrix encoding the roll, pitch, yaw rotation
%
% Inputs:
% roll = desired angle of rotation about X in radians
% pitch = desired angle of rotation about Y in radians
% yaw = desired angle of rotation about Z in radians
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function R = rpy2Rot(roll, pitch, yaw)
R = rotZ(yaw) * rotY(pitch) * rotX(roll);
end
