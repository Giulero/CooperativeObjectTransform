function e = getAngularErrors(R, R_des, om, om_des, e)
e.r = 0.5*scap(R_des'*R - R'*R_des);
e.om = om - R'*R_des*om_des;
end

