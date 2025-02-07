clear; clc; close all;

%% Set Simulation Parameters
enable_control = true;
drawCSM = true;  % Draws CSM on the plot, requires a points3D.mat file to exist
Velocity_Mode = true; % Enables theta_dot only based control
Sim_Exact = false; % Sets if the simulated geometry is exact (true) or slightly different (false)
simTime = 10; % Run the simulation for the specified number of seconds
makeMovie = true;
uses_geometry = true;  % If true it makes a link-list for use in your functions

%% Load ABB Arm Geometry
if uses_geometry
    assert(exist('velocityJacobian.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('transError.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('cpMap.m','file')==2,'Simulation Error:  Need to add project files to path');
    % assert(exist('newtonEuler.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('dhFwdKine.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('constAccelInterp.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('createLink.m','file')==2,'Simulation Error:  Need to add project files to path');
    run('Geometry.m');
end

%% Define Twist Space Trajectory
load points3D;
[times, quats, poss] = getQuatTrajectory(points3D, 0.4);
trajectory = [times, quats, poss];

if drawCSM
    assert(exist('points3D.mat','file')==2,'Simulation Error: Need to make sure a file points3D.mat that has the transformed 2D points in it is on the path');
end

if Velocity_Mode
    assert(exist('velocityJacobian.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('transError.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('cpMap.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('dhFwdKine.m','file')==2,'Simulation Error:  Need to add project files to path');
end

%% Controller Parameter Definitions
Sim_Name = 'System_Twist_Space';
% run Geometry.m; % creates a linkList in the workspace for you and for the simulation
if drawCSM
    load points3D % loads the CSM trajectory points
    sizeCSM = size(points3D,1);
else
    sizeCSM = 0;
end
open([Sim_Name '.slx']);

%% ABB Kp and Kd gains
% If you're going to tweak, make sure you save these initial values.
Kp = [500;500;500;300;10;10];
Kd = [75;75;75;40;1;1];
Ki = [0;500;300;300;0;0];
Ki = Ki * 0;

%% Controller Gains
KpRot = 113;
KpDisp = 646;
KpC = [KpRot;KpRot;KpRot;KpDisp;KpDisp;KpDisp];


%% Plot Titles
theta_err_title = "Theta Errors";
trace_title = "CSM Tracing, ABB Arm";


%% Get num points for simulink
traj_length = size(trajectory,1);

%% Choose Simulation System (perfect model or realistic model)
set_param([Sim_Name '/Theta Controller/ABB Arm Dynamics/sim_exact'], 'sw', int2str(Sim_Exact))

%% Enable/Disable Velocity Mode (Ignores Desired Theta Values)
set_param([Sim_Name '/Theta Controller/Velocity_Mode'], 'sw', int2str(~Velocity_Mode))

%% Enable/Disable Control
set_param([Sim_Name '/Theta Controller/control_enable'], 'sw', int2str(enable_control))

%% Run Simulation
simOut =  sim(Sim_Name,'SimulationMode','normal','AbsTol','1e-5','StopTime', int2str(simTime),...
    'SaveState','on','StateSaveName','xout',...
    'SaveOutput','on','OutputSaveName','yout',...
    'SaveFormat', 'array');

%% Extract Variables From Simulation
laser_tracking = simOut.get('laser_tracking');
theta_dot_actual = simOut.get('theta_actual');
theta_actual = simOut.get('theta_actual');
control_torque = simOut.get('control_torque');
theta_desired = simOut.get('theta_desired');
theta_error = simOut.get('theta_error');
pose_error = simOut.get('pose_error');
laser_pose_error = simOut.get('laser_pose_error');

%% Plot theta as a function of time
figure(1)
hold on;
for i=1:6
    subplot(3,2,i)
    plot(theta_actual.time,theta_actual.signals.values(:,i))
    title(['\theta_', int2str(i)])
    xlabel('time (s)')
    ylabel('angle (rad)')
    grid on;
end
sgtitle("Actual Theta Values")
hold off;

% %% Plot theta desired as a function of time
% figure(3)
% for i=1:6
%     subplot(3,2,i)
%     plot(theta_desired.time,theta_desired.signals.values(:,i))
%     title(['\theta_', int2str(i)])
%     xlabel('time (s)')
%     ylabel('angle (rad)')
%     grid on;
% end
% sgtitle("Desired Theta Values")
% hold off;


% %% Plot theta error as a function of time
% figure(4)
% for i=1:6
%     subplot(3,2,i)
%     plot(theta_error.time,theta_error.signals.values(:,i))
%     title(['\theta_', int2str(i)])
%     xlabel('time (s)')
%     ylabel('angle (rad)')
%     grid on;
% end
% sgtitle(theta_err_title)
% hold off;


%% Plot control torque as a function of time
figure(5)
hold on;
for i=1:6
    subplot(3,2,i)
    plot(control_torque.time,control_torque.signals.values(:,i))
    title(['\theta_', int2str(i)])
    xlabel('time (s)')
    ylabel('angle (rad)')
    grid on;
end
sgtitle("Control torque")
hold off;


%% Plot pose error 
figure(6)
hold on;
subplot(2,1,1)
semilogy(laser_pose_error.time,sqrt(sum(pose_error.data(:,1:3).^2,2)));
title(['Position Pose Error']);
xlabel('time (s)');
xlim([0, simTime])
ylabel('Norm Error (m)');
grid on;
subplot(2,1,2)
semilogy(laser_pose_error.time,sqrt(sum(pose_error.data(:,4:6).^2,2)));
title(['Angular Pose Error']);
xlabel('time (s)');
xlim([0, simTime])
ylabel('Norm Error (rad)');
grid on;
saveas(gcf,'Pose Error','jpg')
hold off;

disp("Sum position pose error: LASER");
disp(sum(sqrt(sum(laser_pose_error.data(:,1:3).^2,2))));

disp("Sum angular pose error: LASER");
disp(sum(sqrt(sum(laser_pose_error.data(:,4:6).^2,2))));

%% Plot control torque
figure(7)
hold on;
subplot(2,1,1)
plot(control_torque.time,control_torque.signals.values);
title(['Control Torque Requested - Saturation Region']);
xlabel('time (s)');
ylabel('Torque [N-m]');
ylim([-350,350])
xlim([0, simTime])
grid on;
subplot(2,1,2)
plot(control_torque.time,control_torque.signals.values);
title(['Control Torque Requested - Full Scale']);
xlabel('time (s)');
ylabel('Torque [N-m]');
xlim([0, simTime])
grid on;
saveas(gcf,'Control Torque','jpg')
hold off;


%% Display Arm Motion Movie
if makeMovie
    obj = VideoWriter('arm_motion','MPEG-4');
    obj.FrameRate = 30;
    obj.Quality = 100;
    obj.open();
end
figure(2)
hold on;
plot(0,0);
ah = gca; % just need a current axis handel
fh = gcf;
stepSize = fix((1/30)/theta_actual.time(2)); % 30 frames per second
% axis([-0.07, 0.16, -0.31, -0.29, 0.39, 0.49])
for i=1:stepSize:length(theta_actual.time)
    plotArm(theta_actual.signals.values(i,:),Sim_Exact,ah);
    hold on;
    grid on;
    plot3(reshape(laser_tracking.signals.values(1,4,1:i),[i,1]),... % x pos
        reshape(laser_tracking.signals.values(2,4,1:i),[i,1]),... % y pos
        reshape(laser_tracking.signals.values(3,4,1:i),[i,1]),'r','Parent',ah); % z pos
    if drawCSM
        plot3(points3D(:,1),points3D(:,2),points3D(:,3),'m','Parent',ah);
    end
    hold off;
    if makeMovie
        obj.writeVideo(getframe(fh));
    end
    pause(1/30)
end
figure(2)
hold on;
title(trace_title)
xlabel('world x [m]')
ylabel('world y [m]')
zlabel('world z [m]')
% Zoomed
axis([-0.07, 0.16, -0.31, -0.29, 0.39, 0.49])
hold off
if makeMovie
    obj.close();
end
