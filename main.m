clc, close all,
clear all;
addpath('tools', 'qpOASES');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cooperative Object Transport in 3D with       % 
% Multiple Quadrotors using                     %
% No Peer Communication                         %
%-----------------------------------------------%
% Course: Multi-robot systems                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% choose the trajectory
% trajectory = "interpolation";
trajectory = "elicoidal";
%% starting error + violated centrosymmetry ?
% start_error = true;
start_error = false;

%%
init % trajectory building

reference_pos = zeros(3,1);
prop1 = [];
prop2 = [];
prop3 = [];
prop4 = [];
propPos = [];
position_error_norm = [];
attitude_error_norm = [];
time = [];
k = 1;
fq = zeros(6,4);

for i=1:(L-1)
    for t=0.0:0.02:(T-0.02)
        % computing cartesian trajectories
        time(k) = t;
        x = computeTrajectory(coeffx, i, t);
        y = computeTrajectory(coeffy, i, t);
        z = computeTrajectory(coeffz, i, t);
        ref_yaw = computeTrajectory(coeffO, i , t);

        r(1:3,k) = [x.p; y.p; z.p];
        ref.p = [x.p; y.p; z.p];
        ref.v = [x.v; y.v; z.v];
        ref.a = [x.a; y.a; z.a];
        ref.j = [x.j; y.j; z.j];
        ref.s = [x.s; y.s; z.s];
        
        %% SE(3) controller
        e = getLinearErrors(ref, h, e);
        FDes = computeFdes(fleet, e, ref);
        RDes = Desired_R(FDes, ref_yaw);
        omegaDes.p = vee(R*RDes.v);
        omegaDes.v = vee(RDes.p'*RDes.a-hat(omegaDes.p)^2);
        e = getAngularErrors(R, RDes.p, omega.p, omegaDes.p, e);
        wrench = computeWrench(fleet, FDes.p, RDes.p, R, omegaDes, omega.p, e);
        des_wrench(1,k) = wrench.f;
        des_wrench(2:4,k) = wrench.tau;  
        position_error_norm(k) = norm(e.p);
        attitude_error_norm(k) = norm(e.r);
        
        %% allocation to quad objects
        for j=1:num_quad
            Wq(j,:) = quadr(j).computeLocalWrench([wrench.f/num_quad; wrench.tau/num_quad]);
        end
        f_quads(:,k) = Wq(:,1);
        tauX_quads(:,k) = Wq(:,2);
        tauY_quads(:,k) = Wq(:,3);
        tauZ_quads(:,k) = Wq(:,4);
        
        TotWrench.f = sum(Wq(:,1));
        TotWrench.tau = [sum(Wq(:,2));sum(Wq(:,3));sum(Wq(:,4))];
        
        %% Evolution of the system
        [h, omega, R] = model(fleet, TotWrench, R, h, omega);
        
        act_wrench(1,k) = TotWrench.f;
        act_wrench(2:4,k) = TotWrench.tau;
        actuated(1:3,k) = h.p;
        error(:,k) = [ref.p] - [h.p];
        k = k + 1;
        %% for visualization
        prop1(:,end+1) = actuated(:,end) + R'*[d;0;0];
        prop2(:,end+1) = actuated(:,end) + R'*[-d;0;0];
        prop3(:,end+1) = actuated(:,end) + R'*[0;d;0];
        prop4(:,end+1) = actuated(:,end) + R'*[0;-d;0];
        
        propPos(:,end+1) = [prop1(:,end);prop2(:,end);prop3(:,end);prop4(:,end)];
    end
end

save('position.mat', 'actuated')
save('propellerPositions.mat', 'propPos')
save('ref.mat', 'r');
%% ------------------------------------
% Visual representation
drawSimulation;

figure(2)
subplot(2,1,1), plot(position_error_norm, 'LineWidth', 1.0), grid on, xlabel('Time/0.02 [s]'), ylabel('||e_p|| [m]');
subplot(2,1,2), plot(attitude_error_norm, 'LineWidth', 1.0), grid on, xlabel('Time/0.02 [s]'), ylabel('||e_R|| [rad]');
% saveas(gcf,'Errors_normal_start_err2_6','epsc')
figure(3)
plot(actuated(1,:), 'LineWidth', 1.0), hold on,  plot(actuated(2,:), 'LineWidth', 1.0), plot(actuated(3,:), 'LineWidth', 1.0),
plot(r(1,:), '--', 'LineWidth', 1.0), hold on,  plot(r(2,:), '--', 'LineWidth', 1.0), plot(r(3,:), '--', 'LineWidth', 1.0),
grid on, xlabel('Time/0.02 [s]'), ylabel('position [m]'),
legend('x', 'y', 'z', 'x_{des}', 'y_{des}', 'z_{des}');
% saveas(gcf,'Positions_normal_start_err2_6','epsc')
