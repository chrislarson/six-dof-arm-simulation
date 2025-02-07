% transError returns a 6x1 vector, where the first 3 elements are position error (desired - current),
% and the last three elements are an angle-axis representation of rotation error.
% Both expressed in the shared base frame.
%
% [error_vector] = transError(Td, Tc)
%
% Outputs:
% error_vector = 6x1 vector describing the error ([pos_error;rot_error]) as expressed in the shared base frame.
%
% Inputs:
% Td = the homogenious matrix describing the desired coordinate pose (of the robot for example)
% in the reference frame (of the world for example).
% Tc = the homogenious matrix describing the current coordinate frame (of the robot for example)
% in the reference frame (of the world for example).
%
% Chris Larson
% Robot Mechanics
% 2023-10-23

function [error_vector] = transError(Td, Tc)
rot_desired = rotFromTransform(Td);
rot_current = rotFromTransform(Tc);
disp_desired = dispFromTransform(Td);
disp_current = dispFromTransform(Tc);

rot_error_vector = rotationError(rot_desired, rot_current);
disp_error_vector = disp_desired - disp_current;

error_vector = [disp_error_vector; rot_error_vector];
end