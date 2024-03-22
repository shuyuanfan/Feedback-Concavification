function fc = fcs2(rho,para)
%FCS simplified concave factor
% created by by Shuyuan Fan, TUHH
% shuyuan.fan@tuhh.de
% fc = (beta*kmin*rho+eta*kmax)/(beta*rho+eta)
kmin = para(1);
kmax = para(2);
beta = para(3);
eta = para(4);
fc = (beta*kmin*rho+eta*kmax)/(beta*rho+eta);
end

