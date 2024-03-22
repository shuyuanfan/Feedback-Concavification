function [K,P,gamma] = DirectHinftyfunction(A,B,Bw,M)
m = size(B',1);
n = size(A,1);
C=sqrtm(M);
X = sdpvar(n,n);
Y = sdpvar(m,n);
gamma = sdpvar(1);
F =[X>=0];
F =[F,gamma>=0];
F = [F,[A*X+X*A'+B*Y+Y'*B' Bw X*C'; Bw' -gamma*eye(m) zeros(m,n); C*X zeros(n,m) -gamma*eye(n)]<=0];
optimize(F,-gamma);
K =value(Y)*inv(value(X));
P = inv(value(X));
gamma = value(gamma);
end