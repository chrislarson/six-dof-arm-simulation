%% drawArm(t1,t2,d3,t4,t5,t6, figureNum) takes in the joint parameters for
% the arm described in HW 6 and draws its configuration in 3d space
%
% t1,t2,d3,t4,t5,t6 - The variable DH parameters for the arm
% TargetPose - the target the arm is trying to reach
% figureNum - the figure number to plot the arm in, if it is not provided,
% a new figure will be generated
%

function drawArm(t1,t2,d3,t4,t5,t6, figureNum, TargetPose)

if ~exist('figureNum','var')
    figureNum = figure();
end
a1 = 1;
a2 = .5;

p0 = [0;0;0];
p1 = p0 + [a1*cos(t1);a1*sin(t1);0];
p2 = p1 + [a2*cos(t1+t2);a2*sin(t1+t2);0];
p3 = p2 + [0;0;d3];

R = [cos(t1 + t2 + t4)*cos(t5)*cos(t6) - sin(t1 + t2 + t4)*sin(t6), - sin(t1 + t2 + t4)*cos(t6) - cos(t1 + t2 + t4)*cos(t5)*sin(t6), -cos(t1 + t2 + t4)*sin(t5);
    cos(t1 + t2 + t4)*sin(t6) + sin(t1 + t2 + t4)*cos(t5)*cos(t6),   cos(t1 + t2 + t4)*cos(t6) - sin(t1 + t2 + t4)*cos(t5)*sin(t6), -sin(t1 + t2 + t4)*sin(t5);
    cos(t6)*sin(t5),                                                -sin(t5)*sin(t6),                    cos(t5)];

L = [p0 p1 p2 p3]';

figure(figureNum)
plot3(L(:,1),L(:,2),L(:,3),'Color','k','LineWidth',2);
hold on;

if exist('TargetPose','var')
    quiver3(ones(3,1)*TargetPose(1,4),ones(3,1)*TargetPose(2,4),ones(3,1)*TargetPose(3,4),TargetPose(1,1:3)',TargetPose(2,1:3)',TargetPose(3,1:3)',.5,'r');
    plot3(TargetPose(1,4),TargetPose(2,4),TargetPose(3,4),'Marker','*','MarkerSize',8,'Color','r');
end

quiver3(ones(3,1)*p3(1),ones(3,1)*p3(2),ones(3,1)*p3(3),R(1,:)',R(2,:)',R(3,:)',.5,'b');

hold off;
axis([-1.5,1.5,-1.5,1.5,-1.5,1.5])


end