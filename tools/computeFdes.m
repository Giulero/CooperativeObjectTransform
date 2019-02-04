function Fdes = computeFdes(fleet, e, r)
kp = fleet.kp;
kv = fleet.kv;
m = fleet.m;
g = fleet.g;
Fdes.p = -kp*e.p - kv*e.v - m*g*[0,0,1]' + m*r.a;
Fdes.v = -kp*e.v - kv*e.a + m*r.j;
Fdes.a = -kp*e.a + m*r.s;
end

