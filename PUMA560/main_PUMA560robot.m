

%% system parameters
%Example 1: PUMA 560 Robot from paper 
% created by by Shuyuan Fan, TUHH
% shuyuan.fan@tuhh.de
% 'Piecewise-linear LQ Control for systems with input constraints'
clc
% system matrix 
A = [zeros(3,3),eye(3,3);
    0 -0.0451 -0.0451 0 0 0;
    0 -0.0457 -0.0457 0 0 0;
    0 -4.5551 -4.5551 0 0 0];

B = [0   0   0;
     0   0   0;
     0   0   0;
     0.0925 0 0.0026;
     0    0.0979  -0.0952;
     0.0026  -0.0954  0.3616];


% system dimention
   m = size(B',1);
   n = size(A,1);

% scheduling parameter
  rho0 = 1043;
  drho = 0.94;
  %drho = 0.94;
  N = 100;

% initial states
x0 = 10/sqrt(6)* [1;1;1;1;1;1];

% if d=0 then no disturbance, if d=1, then there is disturbance
d = 0;

% 

% intial weight matrices
Q0 = eye(6,6);


% maximum input
umax = [97.8 136.4 89.4];

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






%% CQR
% Baseline R
R0q =  Rall(1:3,1:3);

% Concave factor parameters
FcqrQq = [0.5,5000,0.1,0.1,...
          0.5,5000,0.1,0.1,...
          0.5,5000,0.1,0.1,...
          0.5,5000,0.1,0.1,...
          0.5,5000,0.1,0.1,...
          0.5,5000,0.1,0.1]';
FcqrRq = [R0q(1,1),R0q(1,1),1,1,...
        R0q(2,2),R0q(2,2),1,1,...
        R0q(3,3),R0q(3,3),1,1]';
Fcqr_q = [FcqrQq;FcqrRq];
Q0q = diag([FcqrQq(2),FcqrQq(6),FcqrQq(2),FcqrQq(10),FcqrQq(14),FcqrQq(18)]);

% Calculate P(rho)
rhospan = linspace(0,1043,100000);
[~,P0q] = lqr(A,B,Q0q,R0q);
Pc0q = reshape(P0q,[n*n,1]);
Pc0_augq = [Pc0q;FcqrQq;FcqrRq];
[~,Pcq]=ode45(@dpdx_PUMA560,rhospan,Pc0_augq);




%% baseline LQR

FCpara= [1,30,0.01,0.1,...
          1,10,0.01,0.2,...
          1,10,0.01,0.2,...
          1,30,0.01,0.1,...
          1,10,0.01,0.2,...
          1,50,0.01,0.1]';

Rb = Rall(1:3,1:3);

%% LQR
Qb = eye(6);
[Kb,Pb] = lqr(A,B,Qb,Rb);


%% Feedback concavification parameters
Fcdirec = [1,100,1,0.001,...
          0.5,100,1,0.001,...
          1,300,1,0.3,...
          1,200,1,0.1,...
          2.5,100,2,0.01,...
          0.5,200,1,0.02];


