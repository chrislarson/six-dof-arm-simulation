% dhFwdKine Returns the forward kinematics of a manipulator with the
% provided DH parameter set.
%
% H = dhFwdKine(linkList,paramList)
%
% Outputs:
% H = homogeneous transformation matrix representation the forward
% kinematics of the manipulator.
%
% Inputs:
% linkList = array of links, each created by createLink.
% paramList = array containing the current state of their joint variables,
% according to the robot's encoders.
%
% Chris Larson
% Robot Mechanics
% 2023-10-17

function H = dhFwdKine(linkList, paramList)
N = length(linkList);
H = eye(4);

for i = 1:N
    L = linkList(i);
    activeValue = paramList(i);
    activeOffset = activeValue - L.offset;
    if L.isRotary == 1
        % Rotary. Active parameter is theta.
        H = H * dhTransform(L.a, L.d, L.alpha, activeOffset);
    elseif L.isRotary == 0
        % Prismatic. Active parameter is d.
        H = H * dhTransform(L.a, activeOffset, L.alpha, L.theta);
    else
        % Static. No active parameters.
        H = H * dhTransform(L.a, L.d, L.alpha, L.theta);
    end
end

end

