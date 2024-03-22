function [Kall,Pall,gammaall,rhoall] = DirectHinftyLMIs(A,B,Bw,Qhatpara,Mb,rhomax,drho,N)
% created by by Shuyuan Fan, TUHH
% shuyuan.fan@tuhh.de
rho0 = rhomax;
rhoall = [];
Kall = [];
Pall = [];
gammaall = [];
%RHinfty = R;
[T,Lam]=eig(Mb);
QHinfty = Qhatpara';
for i = 0:N+1
    if i==0
        rho =0;
    else
        rho = rho0*drho^(N+1-i);
    end
    q1 = fcs2(rho,QHinfty(1:4));
    q2 = fcs2(rho,QHinfty(5:8));
    q3 = fcs2(rho,QHinfty(9:12));
    q4 = fcs2(rho,QHinfty(13:16));
    q5 = fcs2(rho,QHinfty(17:20));
    q6 = fcs2(rho,QHinfty(21:24));
    Q = diag([q1,q2,q3,q4,q5,q6]);
    %Qhat = sqrtm(Q);
    %Rhat = [zeros(3,3);sqrtm(RHinfty)];
    %C = Qhat;
    M = T'*Q*T*Mb;
    [K,P,gamma] = DirectHinftyfunction(A,B,Bw,M);
    Kall = cat(1,Kall,K);
    Pall = cat(1,Pall,P);
    gammaall = cat(1,gammaall,gamma);
    rhoall = cat(1,rhoall,rho);
end
end