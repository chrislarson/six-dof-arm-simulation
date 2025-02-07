% dhInvKine Returns the parameter list, according to the robot’s encoders, necessary to achieve a
% desired homogenous transform and the residual error in that transform.
% Note: All rotary joints wrapped to the -pi to pi range.
%
% [paramList, error] = dhInvKine(linkList, desTransform, paramListGuess)
%
% Outputs:
% paramList = parameter list necessary to achieve the desired homogenous transform
% error = residual error in the desired homogenous transform
%
% Inputs:
% linkList = a list of the joint parameters created by createLink
% desTransform = the desired homogenous transform
% paramListGuess = an initial guess at the parameters, according to the robot’s encoders.
% Note: Possibly the current arm state
%
% Chris Larson
% Robot Mechanics
% 2023-10-23

function [paramList, error] = dhInvKine(linkList, desTransform, paramListGuess)
paramList = paramListGuess;
Tc = dhFwdKine(linkList,paramList);
error = transError(desTransform,Tc);
dp = Inf;
tol = 1e-15;
while norm(error) > tol && norm(dp) > tol
    Jv = velocityJacobian(linkList,paramList);
    [U,S,V] = svd(Jv);
    S_t = pinv(S,0.001);
    Jv_inv = V*S_t*U';
    dp = Jv_inv*error;
    paramList = paramList + dp;
    Tc = dhFwdKine(linkList,paramList);
    error = transError(desTransform,Tc);
end
end