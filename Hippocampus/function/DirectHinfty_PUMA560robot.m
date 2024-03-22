
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
m = size(B',1);
n = size(A,1);


Q = eye(6);
R = diag([1/97.8,1/136.4,1/89.4]);
 Rhat = [zeros(3,3);]

[Kb,Pb] = lqr(A,B,Q,R);
Mb = -((A-B*Kb)'*Pb+Pb*(A-B*Kb));
C=sqrtm(Mb);
X = sdpvar(n,n);
Y = sdpvar(m,n);
gamma = sdpvar(1);
F =[X>=0];
F =[F,gamma>=0];
F = [F,[A*X+X*A'+B*Y+Y'*B' B X*C'; B' -gamma*eye(m) zeros(m,n); C*X zeros(n,m) -gamma*eye(n)]<=0];
optimize(F,-gamma)

K =value(Y)*inv(value(X))