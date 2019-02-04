function plotEvolution(r, actuated, p_x, p_y, p_z)
% clf
plot3(r(1,:),r(2,:),r(3,:)),grid on, hold on, axis equal,
plot3(actuated(1,:), actuated(2,:), actuated(3,:));
plot3(p_x, p_y, p_z, '*')
legend('reference', 'actuated', 'points');
drawnow;
end

