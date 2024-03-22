function [K,P,gamma] = CQRHinftyfunction(A,B,Bw,Qhat,Rhat)
m = size(B',1);
n = size(A,1);
X = sdpvar(n,n);
Y = sdpvar(m,n);
gamma = sdpvar(1);
F =[X>=0];
F =[F,gamma>=0];
F = [F,[A*X+X*A'+B*Y+Y'*B' Bw X*Qhat'+Y'*Rhat'; Bw' -gamma*eye(m) zeros(m,n); Qhat*X+Rhat*Y zeros(n,m) -gamma*eye(n)]<=0];
optimize(F,-gamma);
K =value(Y)*inv(value(X));
P = inv(value(X));
gamma = value(gamma);
end