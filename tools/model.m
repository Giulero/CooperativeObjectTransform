function [h, om, R] = model(fleet, wrench, R, h, om)


int =  [0 0.02];
e_3 = [0;0;1];
g = fleet.g;
m = fleet.m;

J = fleet.J;

temp = @(t) (g*e_3 - 1/m*(R*wrench.f*e_3));
h.v = integral(temp, int(1), int(2), 'ArrayValued', true) + h.v;

temp1 = @(t)  h.v;
h.p = integral(temp1,int(1),int(2), 'ArrayValued', true) + h.p;

temp2 = @(t)(J\wrench.tau - J\cap(om.p)*J*om.p); %inv(J)*tau - inv(J)*cap(om)*J*om
om.p = integral(temp2,int(1),int(2), 'ArrayValued', true) + om.p;

temp3 = @(t) (R*cap(om.p));
R = integral(temp3,int(1),int(2), 'ArrayValued', true) + R;


end

