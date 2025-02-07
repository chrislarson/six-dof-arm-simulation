clear; clc; close all;

%% Set Simulation Parameters
enable_control = true;
drawCSM = true;  % Draws CSM on the plot, requires a points3D.mat file to exist
Velocity_Mode = false; % Enables theta_dot only based control
Sim_Exact = false; % Sets if the simulated geometry is exact (true) or slightly different (false)
simTime = 10; % Run the simulation for the specified number of seconds
makeMovie = false;
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

%% Define Theta space Trajectory
load points3D;
[times, thetas] = getThetaTrajectory(points3D);
trajectory = [times, thetas];

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
Sim_Name = 'System_Theta_Space';
%run Geometry.m; % creates a linkList in the workspace for you and for the simulation
if drawCSM
    load points3D % loads the CSM trajectory points
    sizeCSM = size(points3D,1);
else
    sizeCSM = 0;
end
open([Sim_Name '.slx']);

%% Define Kp and Kd gains
% If you're going to tweak, make sure you save these initial values.
Kp = [500;500;500;300;10;10];
Kd = [75;75;75;40;1;1];
Ki = [0;500;300;300;0;0];

% Boost proportional gain
Kp = Kp * 20;
Ki = Ki * 5;

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

%% Plot theta as a function of time
figure(1)
for i=1:6
    subplot(3,2,i)
    plot(theta_actual.time,theta_actual.signals.values(:,i))
    title(['\theta_', int2str(i)])
    xlabel('time (s)')
    ylabel('angle (rad)')
    grid on;
end
sgtitle("Actual Theta Values")

%% Plot theta desired as a function of time
figure(3)
for i=1:6
    subplot(3,2,i)
    plot(theta_desired.time,theta_desired.signals.values(:,i))
    title(['\theta_', int2str(i)])
    xlabel('time (s)')
    ylabel('angle (rad)')
    grid on;
end
sgtitle("Desired Theta Values")


%% Plot theta error as a function of time
figure(4)
for i=1:6
    subplot(3,2,i)
    plot(theta_error.time,theta_error.signals.values(:,i))
    title(['\theta_', int2str(i)])
    xlabel('time (s)')
    ylabel('angle (rad)')
    grid on;
end
sgtitle(theta_err_title)

%% Display Arm Motion Movie
if makeMovie
    obj = VideoWriter('arm_motion','MPEG-4');
    obj.FrameRate = 30;
    obj.Quality = 100;
    obj.open();
end
figure(2)
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
title(trace_title)
xlabel('world x [m]')
ylabel('world y [m]')
zlabel('world z [m]')
% Zoomed
axis([-0.07, 0.16, -0.31, -0.29, 0.39, 0.49])
if makeMovie
    obj.close();
end
