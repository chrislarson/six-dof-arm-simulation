clear all; clc;

load 'points3D';

trace_time = 1.5;
time_step = trace_time / 25.0;

[times, quats, poss] = getQuatTrajectory(points3D, time_step);
traj = [times, quats, poss]

N = length(times);

for i=1:N 
    [twist_des, T_des, a_des] = getTwistDesired(traj, i, 0.2);
end










