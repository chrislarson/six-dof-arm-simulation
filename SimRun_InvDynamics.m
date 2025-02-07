clear; clc; close all;

%% Set Simulation Parameters
enable_control = true;
drawCSM = true;  % Draws CSM on the plot, requires a points3D.mat file to exist
Sim_Exact = false; % sets if the simulated geometry is exact (true) or slightly different (false)
trace_time = 3;
Laser_Feedback = true;
makeMovie = true;
plot_arm = false; % controls if the arm is plotted during simulation. Slows down simulink a bit...
zoomView = true;

%% Verify Environment
assert(exist('velocityJacobian.m','file')==2,'Simulation Error:  Need to add project files to path');
assert(exist('transError.m','file')==2,'Simulation Error:  Need to add project files to path');
assert(exist('cpMap.m','file')==2,'Simulation Error:  Need to add project files to path');
assert(exist('newtonEuler.m','file')==2,'Simulation Error:  Need to add project files to path');
assert(exist('dhFwdKine.m','file')==2,'Simulation Error:  Need to add project files to path');
assert(exist('constAccelInterp.m','file')==2,'Simulation Error:  Need to add project files to path');
assert(exist('createLink.m','file')==2,'Simulation Error:  Need to add project files to path');

%% Controller Parameter Definitions
run Geometry.m; % creates a linkList in the workspace for you and for the simulation


%% CSM Points and Trace
load points3D.mat;
trace_time = trace_time * 1.0;
time_step = trace_time / (length(points3D)-1);
simTime = trace_time + 2.5; % Run the simulation for the specified number of seconds

%% Define Boundary Conditions
boundary_conditions = struct;
boundary_conditions.base_angular_velocity = zeros(3,1);
boundary_conditions.base_angular_acceleration = zeros(3,1);
boundary_conditions.base_linear_acceleration = [0;0;9.81];
boundary_conditions.distal_force = zeros(3,1);
boundary_conditions.distal_torque = zeros(3,1);

%% Define System
A = [0, 1; 0, 0];
B = [0; 1]; 
C = [1 0];
D = 0;
ol_poles = eig(A);
ol_sys = ss(A,B,C,D);

%% Compute Kp and Kd gains
Ts = time_step * 0.75;
Tr = Ts * 0.2;
os = 0.05;

% Continuous time design - compute poles
zeta = sqrt(log(os)^2 / (pi^2 + log(os)^2));
wn_ts = 4.6/(zeta * Ts);
wn_tr = (1.53 + 2.31*zeta^2) / Tr;
wn = max(wn_ts, wn_tr);
sigma = zeta*wn;
    
% Continuous time design - compute poles
cl_poles = [
    -wn*(zeta+sqrt(1-zeta^2)*1j);
    -wn*(zeta-sqrt(1-zeta^2)*1j);
];
K = place(A,B,cl_poles);

% Closed-loop system model - continuous time
% A_cl = A-B*K;
% B_cl = B*(K-(B'*B)^-1*B'*A)*[1;0];
% cl_sys = ss(A_cl, B_cl, C, D);
% cl_step_info = stepinfo(cl_sys);

% Discrete time design
dt = 0.001;
Ad = expm(A*dt);
Bd = A'*(Ad-eye(size(A)))*B;
cl_poles_disc = exp(cl_poles*dt);
Kdiscrete = place(Ad, Bd, cl_poles_disc);

Kp = Kdiscrete(1,1);
Kd = Kdiscrete(1,2);

Kp = Kp * 2;
Kd = Kd;


% Closed-loop system model - discrete time
Ad_cl = Ad-Bd*Kd;
Bd_cl = Bd*(Kd-(B'*B)^-1*B'*A)*[1;0];
dcl_sys = ss(Ad_cl, Bd_cl, C, D, dt);
dcl_step_info = stepinfo(dcl_sys);

[Gm,Pm,Wgm,Wpm] = margin(dcl_sys);


%% Get Trajectory
[times, quats, poss] = getQuatTrajectory(points3D, time_step);
trajectory = [times, quats, poss];
traj_length = size(trajectory,1);
trans_percent = 0.1;

%% Controller Parameter Definitions
Sim_Name = 'System_InvDynamics';
if drawCSM
    sizeCSM = size(points3D,1);
else
    sizeCSM = 0;
end
open([Sim_Name '.slx']);

%% Enable/Disable Laser Feedback
set_param([Sim_Name '/Laser_Feedback'],'sw', int2str(Laser_Feedback));

%% Set Gains
set_param([Sim_Name '/Kp'],'Gain',['[' num2str(reshape(Kp,[1,numel(Kp)])) ']']);
set_param([Sim_Name '/Kd'],'Gain',['[' num2str(reshape(Kd,[1,numel(Kd)])) ']']);

%% Choose Simulation System (perfect model or realistic model)
set_param([Sim_Name '/ABB Arm Dynamics/sim_exact'], 'sw', int2str(Sim_Exact))

%% Enable/Disable Control
set_param([Sim_Name '/control_enable'], 'sw', int2str(enable_control))

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
pose_error = simOut.get('pose_error');
% laser_pose_error = simOut.get('laser_pose_error');

%% Plot Titles
theta_err_title = "Theta Errors";
trace_title = "CSM Tracing, 2.5 seconds, Inverse Dynamics Controller, ABB Arm";

%% Plot Theta(t)
% figure(1)
% hold on;
% for i=1:6
%     subplot(3,2,i)
%     plot(theta_actual.time,theta_actual.signals.values(:,i))
%     title(['\theta_', int2str(i)])
%     xlabel('time (s)')
%     ylabel('angle (rad)')
%     grid on;
% end
% sgtitle("Actual Theta Values")
% saveas(gcf,'Theta Set','jpg')
% hold off;

%% Plot Pose Error
figure(2)
hold on;
subplot(2,1,1)
semilogy(pose_error.time,sqrt(sum(pose_error.signals.values(:,1:3).^2,2)));
title(['Position Pose Error, 2.5s trace']);
xlabel('time (s)');
ylabel('Norm Error (m)');
grid on;
subplot(2,1,2)
semilogy(pose_error.time,sqrt(sum(pose_error.signals.values(:,4:6).^2,2)));
title(['Angular Pose Error, 2.5s trace']);
xlabel('time (s)');
ylabel('Norm Error (rad)');
grid on;
saveas(gcf,'Pose Error','jpg')
hold off;

disp("RMS angular pose error: LASER");
ang_pose_err_sums = sum(pose_error.signals.values(:,4:6).^2,2);
mean_ang_pose_err = sum(ang_pose_err_sums)/length(pose_error.signals.values);
rmse_ang_pose = sqrt(mean_ang_pose_err);
disp(rmse_ang_pose);


disp("RMS position pose error: LASER");
pos_pose_err_sums = sum(pose_error.signals.values(:,1:3).^2,2);
mean_pos_pose_err = sum(pos_pose_err_sums)/length(pose_error.signals.values);
rmse_pos_pose = sqrt(mean_pos_pose_err);
disp(rmse_pos_pose);
 


disp("RMS control torque full: LASER");
control_torque_sums = sum(control_torque.signals.values(:,:).^2,2);
mean_control_torque = sum(control_torque_sums)/length(control_torque.signals.values);
rmsc_control_torque = sqrt(mean_control_torque);
disp(rmsc_control_torque);

disp("RMS control torque saturation: LASER");
control_torque_bounded_sums = sum(min(control_torque.signals.values(:,:),350).^2,2);
mean_control_torque_bounded = sum(control_torque_bounded_sums)/length(control_torque.signals.values);
rmsc_control_torque_bounded = sqrt(mean_control_torque_bounded);
disp(rmsc_control_torque_bounded);

%% Plot Control Torque(t)
figure(3)
hold on;
subplot(2,1,1)
plot(control_torque.time,control_torque.signals.values);
title(['Control Torque Requested - Saturation Region, 2.5s trace']);
xlabel('time (s)');
ylabel('Torque [N-m]');
ylim([-350,350])
grid on;
subplot(2,1,2)
plot(control_torque.time,control_torque.signals.values);
title(['Control Torque Requested - Full Scale, 2.5s trace']);
xlabel('time (s)');
ylabel('Torque [N-m]');
grid on;
saveas(gcf,'Control Torque','jpg')
hold off;


%% Display Arm Motion Movie
if makeMovie
    obj = VideoWriter('arm_motion','MPEG-4');
    obj.FrameRate = 30;
    obj.Quality = 50;
    obj.open();
end

%% Plot Arm
figure(4)
hold on;
plot(0,0);
ah = gca; % just need a current axis handel
fh = gcf;
stepSize = fix((1/30)/theta_actual.time(2)); % 30 frames per second
for i=1:stepSize:length(theta_actual.time)
    plotArm(theta_actual.signals.values(i,:),Sim_Exact,ah);
    hold on;
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
if zoomView
    figure(4)
    hold on;
    % Zoomed
    axis([-0.07, 0.16, -0.31, -0.29, 0.39, 0.49])
    % % axis([-0.07, 0.16, 0,0 , 0.39, 0.49])

    hold off
end
if makeMovie
    obj.close();
end
saveas(gcf,'Trace_Overall2.5s trace','jpg')
hold off;


tester = copy(gcf);
hold on;
axis([-0.07, 0.16, -0.31, -0.29, 0.39, 0.49])
hold off
saveas(tester,'Trace_Zoomed2.5s trace','jpg')



tester2 = copy(gcf);
hold on;
axis([-0.07, 0.16, -0.31, -0.29, 0.39, 0.49])
view(0,0)
hold off
saveas(tester2,'Trace_Zoomed_Front2.5s trace','jpg')




