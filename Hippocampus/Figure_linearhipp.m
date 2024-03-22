%% 
% created by by Shuyuan Fan, TUHH
% shuyuan.fan@tuhh.de

% figure of LQR controller
figure(1)
plot(out.LQRstate.Time,out.LQRstate.Data(1,:),'k','LineWidth',2);
hold on
plot(out.LQRstate.Time,out.LQRstate.Data(2,:),'b','LineWidth',2);
hold on
plot(out.LQRstate.Time,out.LQRstate.Data(3,:),'r','LineWidth',2);
hold on
xlabel('Time(s)','fontsize',20)
ylabel('$\Theta$','interpreter','latex','fontsize',20)
set(gca,'xtick',0:10:51,'color',[1,1,1],'LineWidth',1,'fontsize',20)
grid on
set(gcf,'Position',[300 300 720 530])
legend('LQR-$\phi$','LQR-$\theta$','LQR-$\psi$','interpreter','latex','NumColumns',1)
legend('boxon')

%% figure of CNF controller
figure(2)
plot(out.CNFstate.Time,out.CNFstate.Data(1,:),'k','LineWidth',2);
hold on
plot(out.CNFstate.Time,out.CNFstate.Data(2,:),'b','LineWidth',2);
hold on
plot(out.CNFstate.Time,out.CNFstate.Data(3,:),'r','LineWidth',2);
hold on
xlabel('Time(s)','fontsize',20)
ylabel('$\Theta$','interpreter','latex','fontsize',20)
set(gca,'xtick',0:10:51,'color',[1,1,1],'LineWidth',1,'fontsize',20)
grid on
set(gcf,'Position',[300 300 720 530])
legend('CNF-$\phi$','CNF-$\theta$','CNF-$\psi$','interpreter','latex','NumColumns',1)
legend('boxon')

%% figure of CQR controller
figure(3)
plot(out.CQRstate.Time,out.CQRstate.Data(1,:),'k','LineWidth',2);
hold on
plot(out.CQRstate.Time,out.CQRstate.Data(2,:),'b','LineWidth',2);
hold on
plot(out.CQRstate.Time,out.CQRstate.Data(3,:),'r','LineWidth',2);
hold on
xlabel('Time(s)','fontsize',20)
ylabel('$\Theta$','interpreter','latex','fontsize',20)
set(gca,'xtick',0:10:51,'color',[1,1,1],'LineWidth',1,'fontsize',20)
grid on
set(gcf,'Position',[300 300 720 530])
legend('CQR-$\phi$','CQR-$\theta$','CQR-$\psi$','interpreter','latex','NumColumns',1)
legend('boxon')

%% figure of PLQR controller
figure(4)
plot(out.PLQRstate.Time,out.PLQRstate.Data(1,:),'k','LineWidth',2);
hold on
plot(out.PLQRstate.Time,out.PLQRstate.Data(2,:),'b','LineWidth',2);
hold on
plot(out.PLQRstate.Time,out.PLQRstate.Data(3,:),'r','LineWidth',2);
hold on
xlabel('Time(s)','fontsize',20)
ylabel('$\Theta$','interpreter','latex','fontsize',20)
set(gca,'xtick',0:10:51,'color',[1,1,1],'LineWidth',1,'fontsize',20)
grid on
set(gcf,'Position',[300 300 720 530])
legend('PLQR-$\phi$','PLQR-$\theta$','PLQR-$\psi$','interpreter','latex','NumColumns',1)
legend('boxon')
