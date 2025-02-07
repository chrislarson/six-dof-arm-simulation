% newtonEuler - Computes inverse dynamics of a serial link manipulator.
%
% [jointTorques, Jv, JvDot] = newtonEuler(linkList, paramList, paramListDot,
%	paramListDDot, boundry_conditions)
%
% Output(s):
%   jointTorques = Nx1 array of joint torques in N*m.
%
% Input(s):
%   linklist = Nx1 array of links.
%   paramList = Nx1 array of current joint positions.
%   paramListDot = Nx1 array of current joint velocities.
%   paramListDDot = Nx1 array of current joint accelerations.
%   boundry_conditions = struct with members:
%       base_angular_velocity [rad/s]
%       base_angular_acceleration [rad/s^2]
%       base_linear_acceleration [m/s^2]
%       distal_force (in tool frame) [N]
%       distal_torque (in tool frame) [N*m]
%
% Chris Larson
% Robot Mechanics
% 2023-11-17

function [jointTorques] = newtonEuler(linkList, paramList, paramListDot, paramListDDot, boundry_conditions)

% Number of joints
numJoints = length(linkList);

% Values to store between loops
list = repmat(struct( 'Zlast', zeros(3,1),...   % Z i-1, rotation axis for link i in base frame
    'Woi', zeros(3,1),...                       % Angular velocity of origin i in base frame
    'doi', zeros(3,1),...                       % Position of Origin i relative to base in base frame
    'Fi', zeros(3,1),...                        % Inertial Force on link i in base frame
    'Ni', zeros(3,1),...                        % Inertial Torque on link i in base frame
    'rii', zeros(3,1),...                       % Displacement from i-1 to com
    'ri1_i',zeros(3,1) ),...                    % Displacemenbt from i to com
    numJoints,1);

% Initialize variables that get propagated forward
Toi = eye(4);                                        % Transform from 0 to joint i
W = boundry_conditions.base_angular_velocity;        % Angluar Velocity in joint frame
Wdot = boundry_conditions.base_angular_acceleration; % Angular Acceleration in joint frame
Vdot = boundry_conditions.base_linear_acceleration;  % Linear acceleration in joint frame

num_static = 0;

for i=1:numJoints % Begin forward iteration from base to tool
    link = linkList(i);
    Ti = dhFwdKine(link, paramList(i-num_static));
    if link.isRotary == -1
        num_static = num_static + 1;
    end
    
    di = Ti(1:3,4);
    Rt = Ti(1:3,1:3)';
    Z = [0;0;1];
    if link.isRotary == 1 % Rotary Link
        Wdot = Wdot + paramListDDot(i)*Z + paramListDot(i)*(cpMap(W)*Z);
        W = W + paramListDot(i)*Z;
        Vdot = Vdot + cpMap(Wdot)*(Ti(1:3,4)) + cpMap(W)*(cpMap(W)*Ti(1:3,4));
    elseif link.isRotary == 0 % Prismatic Link
        Vdot = Vdot + cpMap(Wdot)*(Ti(1:3,4)) + cpMap(W)*(cpMap(W)*Ti(1:3,4)) + cpMap(2*paramListDot(i)*W)*Z + paramListDDot(i)*Z; % update accel in joint frame
    else % Static Link
        Vdot = Vdot + cpMap(Vdot)*di + cpMap(W)*(cpMap(W)*di); % update accel in joint frame
    end
    
    % Rotate from i-1 frame to i frame
    Wdot = Rt*Wdot;
    W = Rt*W;
    Vdot = Rt*Vdot;
    
    ri1_i =  Rt*di+link.com;
    Vcdot = Vdot + cpMap(Wdot)*link.com + cpMap(W)*(cpMap(W)*link.com);
    
    % Calculate and Save Inertial Force and Torque in the i'th frame
    F = link.mass*Vcdot;
    I = link.inertia;
    N = I*Wdot + cpMap(W)*(I*W);
    
    % Save values specific to calculating Jv and JvDot that we already know
    list(i).Zlast  = Toi(1:3,3);
    Toi = Toi*Ti;
    list(i).doi = Toi(1:3,4);
    list(i).Woi = Toi(1:3,1:3)*W;
    list(i).Fi = Toi(1:3,1:3)*F;
    list(i).Ni = Toi(1:3,1:3)*N;
    list(i).ri1_i = Toi(1:3,1:3)*ri1_i;
    list(i).rii   = Toi(1:3,1:3)*link.com;
end % End Forward Iterations

% Initialize variables for force/torque propagation
f = boundry_conditions.distal_force; % Initialize force to external force on the tool in the tool frame
n = boundry_conditions.distal_torque; % Initialize torque to external torque on the tool in the tool frame

% Rotate f & n to base frame
Roi = Toi(1:3,1:3);
f = Roi*f;
n = Roi*n;

% preallocate joint torque vector
jointTorques = zeros(numJoints,1); % preallocate for speed

for i = numJoints:-1:1 % From Last joint to base
    n = n - cpMap(list(i).rii)*f + list(i).Ni;
    
    % Update Force on joint i in base frame with inertial force from before
    f = f + list(i).Fi;
    
    % Final Update Torque on joint i in base frame with inertial torque and constraint force
    n = n + cpMap(list(i).ri1_i)*f;
    
    if linkList(i).isRotary == 1 % Rotational Joint
        jointTorques(i-num_static) = dot(list(i).Zlast,n);
    elseif linkList(i).isRotary == 0 % Prismatic Joint
        jointTorques(i-num_static) = dot(list(i).Zlast,f);
    else % Static Joint
        num_static = num_static-1;
    end
    
end % end Backward Iterations
end % end newtonEuler Function definition