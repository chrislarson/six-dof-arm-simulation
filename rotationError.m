% rotationError Returns an angle*axis vector, expressed in the reference frame,
% describing what rotation is necessary to take Rot_current to Rot_desired.
%
% rot_error_vector = rotationError(Rot_desired, Rot_current)
%
% Outputs:
% rot_error_vector =  3x1 vector describing the axis of rotation multiplied by the angle of rotation
% (in radians) necessary to transform Rot_current into Rot_desired. See angleAxis2Rot for more info.
%
% Inputs:
% Rot_desired = the rotation matrix describing the desired coordinate frame (of the robot for example)
% in the reference frame (of the world for example).
% Rot_current = the rotation matrix describing the current coordinate frame (of the robot for example)
% in the reference frame (of the world for example).
%
% Chris Larson
% Robot Mechanics
% 2023-10-23

function rot_error_vector = rotationError(Rot_desired, Rot_current)
rot_error = Rot_desired * Rot_current';
rot_error_vector = rot2AngleAxis(rot_error);
end