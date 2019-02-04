function traj = computeTrajectory(coeff,i,t)
% position
traj.p = coeff(i,:)*[t^5 t^4 t^3 t^2 t 1]';
% velocity
traj.v = coeff(i,:)*[5*t^4 4*t^3 3*t^2 2*t 1 0]';
% trajectory
traj.a = coeff(i,:)*[20*t^3 12*t^2 6*t 2 0 0]';
% jerk
traj.j = coeff(i,:)*[60*t^2 24*t 6 0 0 0]';
% snap
traj.s = coeff(i,:)*[120*t 24 0 0 0 0]';
end

