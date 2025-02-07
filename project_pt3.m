load 'points3D';

[times, thetas] = getThetaTrajectory(points3D);
figure(10);
plot(times(1:25), thetas(1:25,:), "-o");

legend("Theta1", "Theta2", "Theta3", "Theta4", "Theta5", "Theta6");
title("Thetas");

% disp(thetas);
% disp(reachables);

N = length(times);
traj = [times, thetas];

disp(traj);

for i=1:N 
    [p, v, a] = constAccelInterp(times(i), traj, 0.4);
end