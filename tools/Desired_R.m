function RD = Desired_R(F_des, yaw_d)
% For position controller define the deisred attitude as follows:

% Define the disired direction of b_1 vector:
% b_1_d = [1;0;0];
% b_1_d_dot=zeros(3,1);
% b_1_d_dot_dot=zeros(3,1);

b_1_d = [cos(yaw_d.p), sin(yaw_d.p),0]';
b_1_d_dot = [-sin(yaw_d.p), cos(yaw_d.p), 0]'*yaw_d.v;
b_1_d_dot_dot = [-cos(yaw_d.p), -sin(yaw_d.p), 0]'*yaw_d.v^2 + [-sin(yaw_d.p), cos(yaw_d.p), 0]'*yaw_d.a;

% Compute the direction of b_3 vector:
A=F_des.p;%-kx*ex-kv*ev-m*g*[0;0;1]+m*xd_2dot;
A_dot=F_des.v;
A_dot_dot=F_des.a;
b_3_c=-A/norm(A);

% Compute the direction of b_2 vector:
C=-cross(b_3_c,b_1_d);
n_C=norm(C);
b_2_c=-C/n_C;

% Compute the direction of b_1 vector:
b_1_c=cross(b_2_c,b_3_c);

% Construct commanded R:
RD.p=[b_1_c,b_2_c,b_3_c];

% Compute the derivative of b_3:
n_A=norm(A);
b_3_c_dot=-(A_dot/n_A)+(A*(A_dot'*A))/(n_A^3);

% Compute the derivative of b_2:
C_dot=-(cross(b_3_c_dot,b_1_d)+cross(b_3_c,b_1_d_dot));
b_2_c_dot=-(C_dot/n_C)+(C*(C_dot'*C))/(n_C^3);

% Compute the derivative of b_1:
b_1_c_dot=cross(b_2_c_dot,b_3_c)+cross(b_2_c,b_3_c_dot);

% Construct derivative of commanded R:
RD.v=[b_1_c_dot,b_2_c_dot,b_3_c_dot];

% Compute the second derivative of b_3:
E0=-A_dot_dot/n_A;
E1=2*(A'*A_dot)*A_dot/(n_A^3);
E2=(norm(A_dot)^2+A'*A_dot_dot)*A/(n_A^3);
E3=-3*(A'*A_dot)^2*A/(n_A^5);
b_3_c_dot_dot=E0+E1+E2+E3;

% Compute the second derivative of b_2:
C_dot_dot=-(cross(b_3_c_dot_dot,b_1_d)+2*cross(b_3_c_dot,b_1_d_dot)+cross(b_3_c,b_1_d_dot_dot));
E0=-C_dot_dot/n_C;
E1=2*(C'*C_dot)*C_dot/(n_C^3);
E2=(norm(C_dot)^2+C'*C_dot_dot)*C/(n_C^3);
E3=-3*(C'*C_dot)^2*C/(n_C^5);
b_2_c_dot_dot=E0+E1+E2+E3;

% Construct the second derivative of commanded R:
RD.a=[cross(b_2_c_dot_dot,b_3_c)+2*cross(b_2_c_dot,b_3_c_dot)+cross(b_2_c,b_3_c_dot_dot),b_2_c_dot_dot,b_3_c_dot_dot];

end
