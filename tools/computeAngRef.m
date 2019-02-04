function R_des = computeAngRef(F_des, yaw_d)

zb = - F_des.p;
b_3_c = zb/norm(zb);

b_1_d = [cos(yaw_d.p);sin(yaw_d.p);0]';

C = -cross(b_3_c, b_1_d);

b_2_c = -C/norm(C);
b_1_c = cross(b_2_c, b_3_c);

R_des.p=[b_1_c,b_2_c,b_3_c'];

end

