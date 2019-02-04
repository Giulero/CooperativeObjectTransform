function coeff = getCoeffs(p_x,T)
    %% Using quintic trajectory
    % at^5+bt^4+ct^3+dt^2+et+f=p(t)

    L = length(p_x);

    % Hessian matrix for the optimization (solve the integral of the jerk)
    % Integral of the jerk is 120^2/3a^2T^3 + 340*24/2abT^2 + 24^2b^2T

    H = zeros(6*(L-1), 6*(L-1));
    H_s = [120^2/3*T^3 240*24/4*T^2;
           240*24*T^2/4 24^2*T];
       
    for i=0:L-2 % iterate on the number of trajectories
       H(i*6+1:(i*6+2),i*6+1:(i*6+2)) = H_s;
    end


    %% need for the constraints

    % constraints on the initial positions on the trajectory

    for i=0:L-2
        A1(i+1,(i*6+1):(i*6+6)) = [0 0 0 0 0 1];
        b1(i+1) = p_x(i+1);
    end
    % constraints on the final positions on the trajectory

    for i=0:L-2
        A2(i+1,(i*6+1):(i*6+6)) = [T^5 T^4 T^3 T^2 T 1];
        b2(i+1) = p_x(i+2);
    end

    % constraints on the intermediate velocities, acceleration, jerk, snap

    for i=0:L-3 % the points exluding the start and the finish
        A3(i+1,(i*6+1):(i*6+12)) = [5*T^4 4*T^3 3*T^2 2*T 1 0 0 0 0 0 -1 0]; % final v - initial v = zero!
        b3(i+1) = 0;
        A4(i+1,(i*6+1):(i*6+12)) = [20*T^3 12*T^2 6*T 2 0 0 0 0 0 -2 0 0]; % final a - initial a = zero!
        b4(i+1) = 0;
        A5(i+1,(i*6+1):(i*6+12)) = [60*T^2 24*T 6 0 0 0 0 0 -6 0 0 0]; % ...
        b5(i+1) = 0;
        A6(i+1,(i*6+1):(i*6+12)) = [120*T 24 0 0 0 0 0 -24 0 0 0 0]; % ...
        b6(i+1) = 0;
    end

    % constraints on the initial and final velocities, acceleration, jerk

    A7 = zeros(6, 6*(L-1));
    b7 = zeros(6,1);
    A7(1,1:6) = [0 0 0 0 1 0]; % initial velocity
    A7(2,end-5:end) = [5*T^4 4*T^3 3*T^2 2*T 1 0]; % final velolcity
    A7(3,1:6) = [0 0 0 2 0 0]; % initial acceleration
    A7(4,end-5:end) = [20*T^3 12*T^2 6*T 2 0 0]; % final acceleration
    A7(1,1:6) = [0 0 6 0 0 0]; % initial jerk
    A7(2,end-5:end) = [60*T^2 24*T 6 0 0 0]; % final jerk

    A = [A1; A2; A3; A4; A5; A6; A7];
    b = [b1'; b2'; b3'; b4'; b5'; b6'; b7];

    options = optimset('Algorithm','interior-point-convex','Display','off');
    coeff_ = quadprog(H, [], [], [], A, b, [], [], [], options);
    
    for i=0:(L-2)
       coeff(i+1,:) = coeff_((6*i+1):(6*i+6))'; 
    end
end

