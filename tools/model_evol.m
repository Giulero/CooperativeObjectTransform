function [h, omega, R] = model_evol(fleet, wrench, R, h, omega)

tspan = [0.0 0.02];

[~,h_] = ode45(@(t,x) dh(t,x,h.v), tspan, h.p);
[~,h_dot_] = ode45(@(t,x) dv(t, x, R, wrench.f, fleet), tspan, h.v);
h.v = h_dot_(end,:)';
h.p = h_(end,:)';

% 
% temp1 = @(t) h_dot;
% dh_ = integral(temp1,0.0,0.02, 'ArrayValued', true);
% h = h + dh_

[~,om_] = ode45(@(t,x) d_om(t, x, wrench.tau, omega.p, fleet), tspan, omega.p);
omega.p = om_(end,:)';
% [~,R_] = ode45(@(t,x) dR(R, om), tspan, R)
dR = integral(@(t)R*cap(omega.p),0.0,0.02, 'ArrayValued', true);
R = R+dR;
end

function dv = dv(t,x,R,f, fleet)
m = fleet.m;
dv = 9.81*[0,0,1]'-1/m*R*f*[0,0,1]'; 
end

function dh = dh(t,x,v)
dh = v;
end

function d_om = d_om(t,x, tau, om, fleet)
J = fleet.J;
d_om = J\(tau - cap(om)*J*om);
end

function dR = dR(R, om)
dR = R*cap(om);
end