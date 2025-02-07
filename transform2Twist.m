% transform2Twist Returns the twist vector corresponding to the provided homogenous transform matrix. The twist should be stacked [v;w th].
%
% t = transform2Twist(H)
%
% Outputs:
% t = twist vector encoding the supplied transformation
%
% Inputs:
% H = homogeneous transformation matrix to be converted into twist vector
%
% Chris Larson
% Robot Mechanics
% 2023-09-02

function t = transform2Twist(H)
R = rotFromTransform(H);
d = dispFromTransform(H);
v = d;

Omega = rot2AngleAxis(R);
theta = norm(Omega);

if theta ~= 0
    k = Omega/theta;
    c1 = 0.5*(sin(theta)/(1-cos(theta)))*eye(3);
    c2_n = 2*(1-cos(theta))-theta*sin(theta);
    c2_d = 2*theta*(1-cos(theta));
    c2 = (c2_n/c2_d)*(k*k');
    c3 = 0.5*cpMap(k);
    v = (c1+c2-c3)*d;
end

t = [v; Omega];
end
