% abbInvKine Returns the joint angles required to reach the desired transformation for the ABB arm.
%
% [th1,th2,th3,th4,th5,th6,reachable] = abbInvKine(T_des, th_last)
%
% Outputs:
% th1 = Joint 1 angle. Real scalar if th_last is supplied; else, 8x1 vector
% with values corresponding to possible solutions.
% th2 = Joint 2 angle. Real scalar if th_last is supplied; else, 8x1 vector
% with values corresponding to possible solutions.
% th3 = Joint 3 angle. Real scalar if th_last is supplied; else, 8x1 vector
% with values corresponding to possible solutions.
% th4 = Joint 4 angle. Real scalar if th_last is supplied; else, 8x1 vector
% with values corresponding to possible solutions.
% th5 = Joint 5 angle. Real scalar if th_last is supplied; else, 8x1 vector
% with values corresponding to possible solutions.
% th6 = Joint 6 angle. Real scalar if th_last is supplied; else, 8x1 vector
% with values corresponding to possible solutions.
% reachable = true if the transform can be achieved, false if it cannot be achieved.
%
% Inputs:
% T_des = The desired transformation for the ABB arm.
% th_last = [optional] 6x1 vector of the last set of thetas used to select a specific solution.
%
% Chris Larson
% Robot Mechanics
% 2023-10-17


function [th1,th2,th3,th4,th5,th6,reachable] = abbInvKine(T_des, th_last)
% DH Parameters
a1 = 0;
a2 = 0.27;
a3 = 0.07;
alpha1 = -pi/2;
d1 = 0.290;
d4 = 0.302;
d6 = 0.072;

T06 = T_des;
th1_ = zeros([8 1]);
th2_ = zeros([8 1]);
th3_ = zeros([8 1]);
th4_ = zeros([8 1]);
th5_ = zeros([8 1]);
th6_ = zeros([8 1]);
errs = zeros([8 7]);
reachable_=ones([8 1]);

i = 0;

d006 = T06(1:3,4);
z006 = T06(1:3,3);
d005 = d006 - (d6 * z006);
d001 = [0; 0; 1] * d1;
d015 =  d005 - d001;
L1 = sqrt(d4^2 + a3^2);
psi = atan2(d4, a3);

theta1 = solveTheta1(d005(1), d005(2));
numTh1 = length(theta1);
for nTh1 = 1:numTh1
    T01 = dhTransform(a1, d1, alpha1, theta1(nTh1));
    R01 = rotFromTransform(T01);
    d115 = R01\d015;
    [theta3, reach] = solveTheta3(psi, a2, L1, d115);
    if reachable_(i+1) == 1 && reach == 0
        reachable_(i+1) = 0;
    end
    numTh3 = length(theta3);
    for nTh3 = 1:numTh3
        theta2 = solveTheta2(theta3(nTh3), d115(1), d115(2), psi, L1, a2);
        R36 = abbRot36(theta1(nTh1), theta2(1), theta3(nTh3), T06);
        [theta5, reach] = solveTheta5(R36);
        if reachable_(i+1) == 1 && reach == 0
            reachable_(i+1) = 0;
        end
        numTh5 = length(theta5);
        for nTh5 = 1:numTh5
            i = i + 1;
            if abs(sin(theta5(nTh5))) < sqrt(eps)
                if cos(theta5(nTh5)) == -1
                    theta4 = 0;
                    theta6 = atan2(R36(2,1), -R36(1,1));
                else
                    theta4 = 0;
                    theta6 = atan2(R36(2,1), R36(1,1));
                end
                if exist("th_last", "var")
                    % Least squares to find better Th4 and Th6
                    theta4 = th_last(4)/2 - th_last(6)/2 + theta6/2;
                    theta6 = -th_last(4)/2 + th_last(6)/2 + theta6/2;
                end
                
            else
                theta4 = atan2(-R36(2,3)/sin(theta5(nTh5)), -R36(1,3)/sin(theta5(nTh5)));
                theta6 = atan2(-R36(3,2)/sin(theta5(nTh5)), R36(3,1)/sin(theta5(nTh5)));
            end
            th1_(i,:) = theta1(nTh1);
            th2_(i,:) = theta2;
            th3_(i,:) = theta3(nTh3);
            th4_(i,:) = theta4;
            th5_(i,:) = theta5(nTh5);
            th6_(i,:) = theta6;
            
            if exist("th_last", "var")
                % Theta 1 correction
                if th1_(i,:) - th_last(1) > pi
                    th1_(i,:) = th1_(i,:) - 2*pi;
                end
                if th1_(i,:) - th_last(1) < -pi
                    th1_(i,:) = th1_(i,:) + 2*pi;
                end
                
                % Theta 2 correction
                if th2_(i,:) - th_last(2) > pi
                    th2_(i,:) = th2_(i,:) - 2*pi;
                end
                if th2_(i,:) - th_last(2) < -pi
                    th2_(i,:) = th2_(i,:) + 2*pi;
                end
                
                % Theta 3 correction
                if th3_(i,:) - th_last(3) > pi
                    th3_(i,:) = th3_(i,:) - 2*pi;
                end
                if th3_(i,:) - th_last(3) < -pi
                    th3_(i,:) = th3_(i,:) + 2*pi;
                end
                
                % Theta 4 correction
                if th4_(i,:) - th_last(4) > pi
                    th4_(i,:) = th4_(i,:) - 2*pi;
                end
                if th4_(i,:) - th_last(4) < -pi
                    th4_(i,:) = th4_(i,:) + 2*pi;
                end
                
                % Theta 5 correction
                if th5_(i,:) - th_last(5) > pi
                    th5_(i,:) = th5_(i,:) - 2*pi;
                end
                if th5_(i,:) - th_last(5) < -pi
                    th5_(i,:) = th5_(i,:) + 2*pi;
                end
                
                % Theta 6 correction
                if th6_(i,:) - th_last(6) > pi
                    th6_(i,:) = th6_(i,:) - 2*pi;
                end
                if th6_(i,:) - th_last(6) < -pi
                    th6_(i,:) = th6_(i,:) + 2*pi;
                end
                
                
            end
            if reachable_(i) == 1 && (~isreal(th1_(i)) || ~isreal(th2_(i)) || ~isreal(th3_(i)) || ~isreal(th4_(i)) || ~isreal(th5_(i)) ||~ isreal(th6_(i)))
                reachable_(i) = 0;
            end
            
            this_err = (([th1_(i), th2_(i), th3_(i), th4_(i), th5_(i), th6_(i)]' - th_last).^2)';
            errs(i,1:6) = this_err;
            errs(i,7) = sum(errs(i,1:6));
        end
    end
end

% Find where error sum is minimized and is reachable
if exist("th_last", "var")
    [~, solnIdx] = min(errs(:,7));
    th1(1) = wrapToPi(th1_(solnIdx));
    th2(1) = wrapToPi(th2_(solnIdx));
    th3(1) = wrapToPi(th3_(solnIdx));
    th4(1) = wrapToPi(th4_(solnIdx));
    th5(1) = wrapToPi(th5_(solnIdx));
    th6(1) = wrapToPi(th6_(solnIdx));
    reachable(1) = reachable_(solnIdx);
else
    th1 = wrapToPi(th1_);
    th2 = wrapToPi(th2_);
    th3 = wrapToPi(th3_);
    th4 = wrapToPi(th4_);
    th5 = wrapToPi(th5_);
    th6 = wrapToPi(th6_);
    reachable = reachable_;
    
end
end

%%%%%%%%%%
% Helpers
%%%%%%%%%%

% Function to solve Theta1
function th1 = solveTheta1(d005x,d005y)
if (d005y > 0)
    d005y = -1 * d005y;
    d005x = -1 * d005x;
end
th1(1) = atan2(d005y, d005x);
th1(2) = th1(1) + pi;
end

% Function to solve Theta2
function th2 = solveTheta2(th3, d115x,d115y, psi, L1, a2)
beta = atan2(d115y, d115x);
epsilon(1) = atan2(L1*sin(th3 + psi), a2+L1*cos(th3 + psi));
th2 = beta - epsilon(1) + pi/2;
end


% Function to solve Theta3
function [th3, reachable] = solveTheta3(psi,a2,L1, d115)
cos_gamma = (a2^2 + L1^2 - norm(d115)^2)/(2*a2*L1);
if (1-cos_gamma < 0 || 1+cos_gamma<0)
    reachable = 0;
else
    reachable = 1;
end
gamma(1) = 2 * atan2(real(sqrt((1-cos_gamma))),real(sqrt((1+cos_gamma))));
gamma(2) = 2 * atan2(-1*real(sqrt((1-cos_gamma))),real(sqrt((1+cos_gamma))));
th3(1) = pi - gamma(1) - psi;
th3(2) = pi - gamma(2) - psi;
end


% Function to solve Theta5
function [th5, reachable] = solveTheta5(R36)
if R36(1,3)^2 + R36(2,3)^2 < 0
    reachable = 0;
else
    reachable = 1;
end
th5(1) = atan2(real(sqrt(R36(1,3)^2 + R36(2,3)^2)), R36(3,3));
th5(2) = atan2(-real(sqrt(R36(1,3)^2 + R36(2,3)^2)), R36(3,3));
end
