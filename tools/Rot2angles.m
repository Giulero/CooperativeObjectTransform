function [roll, pitch, yaw] = Rot2angles(R)
roll = atan2(R(3,2),R(3,3));
yaw = atan2(R(2,1),R(1,1));
pitch = atan2(-R(3,1), cos(yaw)*R(1,1) + sin(yaw)*R(2,1));
end

