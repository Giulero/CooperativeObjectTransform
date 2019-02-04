function wrench = computeWrench(fleet, F_des, R_des, R, om_des, om, e)

J = fleet.J;
kR = fleet.kR;
kom = fleet.kom;
wrench.f = -dot(F_des,R*[0,0,1]');
wrench.tau = -kR*e.r - kom*e.om + cap(om)*J*om + J*(-cap(om)*R'*R_des*om_des.p + R'*R_des*om_des.v);
end

