function plotEvolutionFleet(r, actuated, p_x, p_y, p_z, R)
clf
plot3(r(1,:),r(2,:),r(3,:)),grid on, hold on
plot3(actuated(1,:), actuated(2,:), actuated(3,:));
plot3(p_x, p_y, p_z, '*')
legend('reference', 'actuated', 'points');
prop1 = actuated(:,end) + R*[0.4;0;0];
prop2 = actuated(:,end) + R*[-0.4;0;0];
prop3 = actuated(:,end) + R*[0;0.4;0];
prop4 = actuated(:,end) + R*[0;-0.4;0];

% plot3(prop1(1), prop1(2), prop1(3), 'o');
% plot3(prop2(1), prop2(2), prop2(3), 'o');
% plot3(prop3(1), prop3(2), prop3(3), 'o');
% plot3(prop4(1), prop4(2), prop4(3), 'o');

drawnow;
end

