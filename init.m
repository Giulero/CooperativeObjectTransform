%% Object properties

fleet.J = [0.17,         0,         0;
              0,      0.17,         0; 
              0,         0,      0.34];

e3 = [0 0 1]';
fleet.g = 9.81; % m/s
fleet.m = 2; % kg

fleet.kp = 15;
fleet.kv = 9;
fleet.kR = 18;
fleet.kom = 14;



%% Optimized trajectory building

%% ------------------------------------------------------------------------
% generic trajectory
if trajectory == "interpolation"
    % time interval between one point and another
    T = 10;
    % waypoints
    p_x =  [0  1  2  3 0 0 0 0];
    p_y =  [0  1  2  3 6 6 6 6];
    p_z =  [0 -1 -2 -1 0 0 0 0];
    p_om = [0  0  0  0 0 0 0 0];
end
%% ------------------------------------------------------------------------
% elicoidal trajectory

if trajectory == 'elicoidal'
    % time interval between one point and another
    T = 1.0; 
    for i=0:100;
       s = min(i-70,0);
       p_z(i+1) = s*0.1;
       p_x(i+1) = cos(-0.3*s);
       p_y(i+1) = sin(-0.3*s);
       p_om(i+1) = 0.0*s;
    end
end

L = length(p_x);

coeffx = getCoeffs(p_x, T);
coeffy = getCoeffs(p_y, T);
coeffz = getCoeffs(p_z, T);
coeffO = getCoeffs(p_om, T);

% quads parameters
num_quad = 4;
d = 0.4;
r = 0.1;
c = 0.01;

%% set initial error
if start_error == false
    h.p = [p_x(1) p_y(1) p_z(1)]';
    for i=0:num_quad-1
       alpha = 2*pi/num_quad*i;
       quadr(i+1) = Quadcopter(d, alpha, c, r);
    end
end
if start_error
    h.p = [p_x(1)+0.2 p_y(1)+0.1 p_z(1)-0.2]'; % initial position with error
    for i=0:num_quad-1
        alpha = 2*pi/num_quad*i + 0.4*rand();
        quadr(i+1) = Quadcopter(d + 0.1*rand(), alpha, c, r);
    end
end
h.v = [0.0 0.0 0.0]'; % initial velocity
h.a = [0.0 0.0 0.0]'; % initial acceleration
h.j = [0.0 0.0 0.0]'; % initial jerk
R = angles2Rot(0.0, 0.0, 0.0); % initial attitude
omega.p = [0.0 0.0 0.0]'; % initial angular velocity
e = struct('p', 0.0, 'v', 0.0, 'a', 0.0, 'j', 0.0, 'r', 0.0, 'om', 0.0); 
