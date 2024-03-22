

%% Pb, Kb, Mb
%A =[];
%B =[];
Fc
Fchalf = sqrtm(Fc);
Pb = P0;
Kb = Kall(1:3,:);
Mb = -((A-B*Kb)'*Pb + Pb*(A-B*Kb));
[Tb,lamMb] = eig(Mb);
Tb = inv(Tb);
Fcstarhat = inv(Tb)*Fchalf*T_b;
Aclb = A - B*Kb;
Kn = inv(B'*B)*B'*(A-Fcstarhat*Aclb*Fcstarhat);
u = -Kn*x;
