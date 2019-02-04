function R = angles2Rot(roll, pitch, yaw)
c1 = cos(roll);
s1 = sin(roll);
c2 = cos(pitch);
s2 = sin(pitch);
c3 = cos(yaw);
s3 = sin(yaw);

R = zeros(3,3);
Rx = [1  0   0;
      0 c1 -s1;
      0 s1  c1;];
Ry = [c2 0 s2;
      0  1   0;
      -s2 0  c1;];
Rz = [c3 -s3 0;
      s3  c3 0;
       0   0 1;];
   
R = Rz*Ry*Rx;
end

