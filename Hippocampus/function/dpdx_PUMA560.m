function dp = dpdx_PUMA560(rho,P)
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

%% Solving Recaati equation
Preal = P(1:n*n);
p_ = reshape(Preal,[n,n]);
p = p_';
F = (Q - p*B*inv(R)*B'*p+ A'*p+p*A);
dp1 = reshape(F',[n*n,1]);
dp2 = zeros((n+m)*4,1);
dp = [dp1;dp2];
end