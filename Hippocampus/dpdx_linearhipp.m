function dp = dpdx_linearhipp(rho,P)
% created by by Shuyuan Fan, TUHH
% shuyuan.fan@tuhh.de
% fcs: concave factor
%FCS simplified concave factor
% fc = (beta*kmin*rho+eta*kmax)/(beta*rho+eta)
% diagnoal Q matrix
%N = size(P,1);
m = 3;
n = 6;

%Fcqrq = zeros(n*4,1);
Fcqrq = P(n*n+1:60);
q = [];
for i=1:6
   ParameterQ = Fcqrq(4*(i-1)+1:4*i);
   q_ = fcs2(rho,ParameterQ);
   q = cat(1,q,q_);
end
Q = diag(q);
Fcqrr = P(61:72);
 r=[];
 for j=1:3
    ParameterR = Fcqrr(4*(j-1)+1:4*j);
    r_ = fcs2(rho,ParameterR);
    r = cat(1,r,r_);
 end
 R = diag(r);

%% system matrix
zg    =  0.05;    % [m]
g     =  9.81;    % [m/s^2]

m1     =  1.43;    % [kg]
Ix    = 0.00241; % [kgm^2]
Iy    = 0.01072; % [kgm^2]
Iz    = 0.01072; % [kgm^2]
Xudot = -1.11;    % [kg]
Yvdot = -2.80;    % [kg]
Zwdot = -2.80;    % [kg]
Kpdot = -0.0018;  % [kgm^2]
Mqdot = -0.0095;  % [kgm^2]
Nrdot = -0.0095;  % [kgm^2]
Xu    = -4.56;    % [kg/m]
Yv    = -17.36;   % [kg/m]
Zw    = -17.36;   % [kg/m]
Kp    = -0.0028;  % [kgm^2]
Mq    = -0.0476;  % [kgm^2]
Nr    = -0.0476;  % [kgm^2]


% Construct rotational tensor
%R = body_frame_rotation(x(4:6));

% Construct velocity transformation tensor
%T = [ 1  sin(phi)*tan(theta)   cos(phi)*tan(theta) ;
 %   0  cos(phi)             -sin(phi)            ;
 %   0  sin(phi)/cos(theta)   cos(phi)/cos(theta) ];
%J = [ R         zeros(3) ;
  %  zeros(3)  T        ];

% Calculate mass matrix
Ma  = -diag([Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot]);
Mrb =  diag([m1,  m1,  m1, Ix, Iy, Iz]);
M = Ma + Mrb;

M1 = M(1,1); % for p
M2 = M(2,2); % for q
M3 = M(3,3); % for r

% Compute hydrostatic load
gravity = zg * g * m1;
%G = [ 0; 0; 0; gravity * cos(theta) * sin(phi); gravity * sin(theta); 0 ];

% Construct coriolis matrix, see Fossen 2011, Theorem 3.2
%C12 = -skew(M(1:3,1:3) * nu(1:3));
%C = [ zeros(3)   C12                        ;
%    C12       -skew(M(4:6,4:6) * nu(4:6)) ];

% Construct damping matrix
%D = -diag([Xu * abs(u), Yv * abs(v),...
%    Zw * abs(w), Kp * abs(p),...
%    Mq * abs(q), Nr * abs(r)]);


% Build state space equations
%tau  = [ in(1); 0; 0; in(2:4) ];

%dx       = zeros(12,1);
%dx(1:6)  = J*nu;
%dx(7:12) = M \ (tau - (C+D)*nu - G);

% for T
A1 = [zeros(3,3),eye(3);
      zeros(3,6)];


A2 = [0 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0;
       -gravity/M1 0 0 0 0 0;
       0 -gravity/M2 0 0 0 0;
       0 0 0 0 0 0];

% for  p q r =0, C=0,D=0

% x1 = phi  x2= theta  x3 = psi  x4 = p  x5 = q  x6 = r
A = A1+A2;
B = [zeros(3,3);diag([1/M1,1/M2,1/M3])];

%% Solving Recaati equation
Preal = P(1:n*n);
p_ = reshape(Preal,[n,n]);
p = p_';
F = (Q - p*B*inv(R)*B'*p+ A'*p+p*A);
dp1 = reshape(F',[n*n,1]);
dp2 = zeros((n+m)*4,1);
dp = [dp1;dp2];
end