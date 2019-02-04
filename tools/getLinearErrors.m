function e = getLinearErrors(ref, act, e)
e.p = act.p - ref.p;
e.v = act.v - ref.v;
e.a = act.a - ref.a;
e.j = act.j - ref.j;
% e_r = 0.5*scap(R_des'*R_act - R_act'*R_des);
% e_om = om_act - R_act'*R_des*om_des;
end

