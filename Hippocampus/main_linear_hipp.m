
%% system parameters
%Example linear Hippocampus: 
% created by by Shuyuan Fan, TUHH
% shuyuan.fan@tuhh.de
% 
clc
% system matrix 
%% linearization for phi=0,theta=0,psi=0,p,q,r=0
% assume u,v,w is the external disturbance

zg    =  0.05;    % [m]
g     =  9.81;    % [m/s^2]

m     =  1.43;    % [kg]
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
Mrb =  diag([m,  m,  m, Ix, Iy, Iz]);
M = Ma + Mrb;

M1 = M(1,1); % for p
M2 = M(2,2); % for q
M3 = M(3,3); % for r

% Compute hydrostatic load
gravity = zg * g * m;
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


% for G   M\(-G)
A2 = [0 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0;
       -gravity/M1 0 0 0 0 0;
       0 -gravity/M2 0 0 0 0;
       0 0 0 0 0 0];
% system dimention

% for  p q r =0, C=0,D=0

% x1 = phi  x2= theta  x3 = psi  x4 = p  x5 = q  x6 = r
A = A1+A2;
B = [zeros(3,3);diag([1/M1,1/M2,1/M3])];
m = size(B',1);
n = size(A,1);


%%
% initial states
x0 = [pi/6;pi/4;pi/3;1;1;1];

% saturated input
umax = [0.0288 0.5772 0.5772];

% basekube Q
Qb = 0.0001*diag([1,1,1,1,1,1]);

%% Design P-LQR controller
 rho0 = 20;
 drho = 0.94;
 N = 100;
% intial weight matrices
Q0 =Qb;

% Rall: R matrices for all rho
 Rall = [];

% Kall: control gain K for all rho
 Kall = [];

% Pall: The solution P for all rho
 Pall = [];

% All scheduling parameters rho
 rho = zeros(N,1);
 rhoall = [];
% calculate K P rho
for i = 1:N+1
    % rho 
    rho(i) = rho0 *drho^(i-1);

    rhoall = cat(1,rhoall,rho(i));
    % initial error > 0.0001
    d = 10;

    % the slected rho
    rhod = rho(i);

    % inital R
    R_ = eye(3,3);

    % iterated R
    ep = zeros(m,1);

    Q_ = Q0;

    % iteration
    while d>0.0001
        [K_,P_] = lqr(A,B,Q_,R_);
        for j = 1:m
            phi = 1/umax(j)*sqrt(B(:,j)'*P_*B(:,j));
            ep(j) = sqrt(rhod)*phi;
        end
        ep_ = diag(ep);

        % calculate the error
        d = norm(ep_-R_);

        % design R
        R_ = ep_;
    end
    % store all datas
    Kall= cat(1,Kall,K_);
    Pall=cat(1,Pall,P_);
    Rall=cat(1,Rall,R_);
end
nK0 = 1;



%% design LQR controller
% fixed R
Rb = Rall(1:3,1:3);
[Kb,Pb] = lqr(A,B,Qb,Rb);

%% design CQR controller
R0q = Rb;

% parameters of the concave facotr Q(\rho) 
FcqrQq = [0.5,5000,0.1,0.1,...
          0.5,5000,0.1,0.1,...
          0.5,5000,0.1,0.1,...
          0.5,5000,0.1,0.1,...
          0.5,5000,0.1,0.1,...
          0.5,5000,0.1,0.1]';

% R is fixed 
FcqrRq = [R0q(1,1),R0q(1,1),1,1,...
        R0q(2,2),R0q(2,2),1,1,...
        R0q(3,3),R0q(3,3),1,1]';
Fcqr_q = [FcqrQq;FcqrRq];

%% Q0 the maximum Q
Q0q = diag([FcqrQq(2),FcqrQq(6),FcqrQq(2),FcqrQq(10),FcqrQq(14),FcqrQq(18)]);

% Calculate P(rho)
rhom = 20;
rhospan = linspace(0,rhom,1000);

% compute P0
[~,P0q] = lqr(A,B,Q0q,R0q);
Pc0q = reshape(P0q,[n*n,1]);
Pc0_augq = [Pc0q;FcqrQq;FcqrRq];

% solve the ODE offline
[~,Pcq]=ode45(@dpdx_linearhipp,rhospan,Pc0_augq);





