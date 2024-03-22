%% state 1
% created by by Shuyuan Fan, TUHH
% shuyuan.fan@tuhh.de
figure(1)

plot(out.PLCstates.Time,out.PLCstates.Data(1,:),'k','LineWidth',1.5);
hold on

plot(out.LQRstates.Time,out.LQRstates.Data(1,:),'g','LineWidth',1.5);
hold on

plot(out.CNFstates.Time,out.CNFstates.Data(1,:),'b','LineWidth',1.5);
hold on

plot(out.CQRstates.Time,out.CQRstates.Data(1,:),'r-o','MarkerIndices',1:500:length(out.CQRstates.Data(1,:)),'LineWidth',1.5);
hold on

plot(out.FCstates.Time,out.FCstates.Data(1,:),'m-+','MarkerIndices',1:500:length(out.FCstates.Data(1,:)),'LineWidth',1.5);
hold on


xlabel('Time(s)','fontsize',20)
ylabel('$x_1$','interpreter','latex','fontsize',20)
set(gca,'xtick',0:1:10,'ytick',0:1:6,'color',[1,1,1],'LineWidth',1,'fontsize',20)
grid on
set(gcf,'Position',[300 300 720 530])
%set(gcf,'Position',[300 300 400 300])
legend('P-LQR','LQR','CNF','CQR','Direct-FC','interpreter','latex','NumColumns',1)
legend('boxon')

%% state 2
figure(2)
plot(out.PLCstates.Time,out.PLCstates.Data(2,:),'k','LineWidth',1.5);
hold on

plot(out.LQRstates.Time,out.LQRstates.Data(2,:),'g','LineWidth',1.5);
hold on

plot(out.CNFstates.Time,out.CNFstates.Data(2,:),'b','LineWidth',1.5);
hold on

plot(out.CQRstates.Time,out.CQRstates.Data(2,:),'r-o','MarkerIndices',1:500:length(out.CQRstates.Data(1,:)),'LineWidth',1.5);
hold on

plot(out.FCstates.Time,out.FCstates.Data(2,:),'m-+','MarkerIndices',1:500:length(out.FCstates.Data(1,:)),'LineWidth',1.5);
hold on


xlabel('Time(s)','fontsize',20)
ylabel('$x_2$','interpreter','latex','fontsize',20)
set(gca,'xtick',0:1:10,'ytick',0:1:6,'color',[1,1,1],'LineWidth',1,'fontsize',20)
grid on
set(gcf,'Position',[300 300 720 530])
%set(gcf,'Position',[300 300 400 300])
legend('P-LQR','LQR','CNF','CQR','Direct-FC','interpreter','latex','NumColumns',1)
legend('boxon')


%% state 3
figure(3)
plot(out.PLCstates.Time,out.PLCstates.Data(3,:),'k','LineWidth',1.5);
hold on

plot(out.LQRstates.Time,out.LQRstates.Data(3,:),'g','LineWidth',1.5);
hold on

plot(out.CNFstates.Time,out.CNFstates.Data(3,:),'b','LineWidth',1.5);
hold on

plot(out.CQRstates.Time,out.CQRstates.Data(3,:),'r-o','MarkerIndices',1:500:length(out.CQRstates.Data(1,:)),'LineWidth',1.5);
hold on


plot(out.FCstates.Time,out.FCstates.Data(3,:),'m-+','MarkerIndices',1:500:length(out.FCstates.Data(1,:)),'LineWidth',1.5);
hold on


xlabel('Time(s)','fontsize',20)
ylabel('$x_3$','interpreter','latex','fontsize',20)
set(gca,'xtick',0:1:10,'ytick',-8:2:8,'color',[1,1,1],'LineWidth',1,'fontsize',20)
grid on
set(gcf,'Position',[300 300 720 530])
%set(gcf,'Position',[300 300 400 300])
legend('P-LQR','LQR','CNF','CQR','Direct-FC','interpreter','latex','NumColumns',1)
legend('boxon')


%% rho
figure(4)
plot(out.PLCrho.Time,out.PLCrho.Data(:,1),'k','LineWidth',1.5);
hold on

plot(out.LQRrho.Time,out.LQRrho.Data(:,1),'g','LineWidth',1.5);
hold on

plot(out.CNFrho.Time,out.CNFrho.Data(:,1),'b','LineWidth',1.5);
hold on

plot(out.CQRrho.Time,out.CQRrho.Data(:,1),'r-o','MarkerIndices',1:500:length(out.CQRstates.Data(1,:)),'LineWidth',1.5);
hold on


plot(out.FCrho.Time,out.FCrho.Data(:,1),'m-+','MarkerIndices',1:500:length(out.FCstates.Data(1,:)),'LineWidth',1.5);
hold on


xlabel('Time(s)','fontsize',20)
ylabel('$\rho=x^\top P_b x$','interpreter','latex','fontsize',20)
set(gca,'xtick',0:1:10,'ytick',0:200:1000,'color',[1,1,1],'LineWidth',1,'fontsize',20)
grid on
set(gcf,'Position',[300 300 720 530])
%set(gcf,'Position',[300 300 400 300])
legend('P-LQR','LQR','CNF','CQR','Direct-FC','interpreter','latex','NumColumns',1)
legend('boxon')


%% sqrt(rho)
figure(5)
plot(out.PLCrho1.Time,out.PLCrho1.Data(:,1),'k','LineWidth',1.5);
hold on

plot(out.LQRrho1.Time,out.LQRrho1.Data(:,1),'g','LineWidth',1.5);
hold on

plot(out.CNFrho1.Time,out.CNFrho1.Data(:,1),'b','LineWidth',1.5);
hold on

plot(out.CQRrho1.Time,out.CQRrho1.Data(:,1),'r-o','MarkerIndices',1:500:length(out.CQRstates.Data(1,:)),'LineWidth',1.5);
hold on


plot(out.FCrho1.Time,out.FCrho1.Data(:,1),'m-+','MarkerIndices',1:500:length(out.FCstates.Data(1,:)),'LineWidth',1.5);
hold on


xlabel('Time(s)','fontsize',20)
ylabel('$\sqrt{\rho}$','interpreter','latex','fontsize',20)
set(gca,'xtick',0:1:10,'ytick',0:10:50,'color',[1,1,1],'LineWidth',1,'fontsize',20)
grid on
set(gcf,'Position',[300 300 720 530])
%set(gcf,'Position',[300 300 400 300])
legend('P-LQR','LQR','CNF','CQR','Direct-FC','interpreter','latex','NumColumns',1)
legend('boxon')

%% sqrt(rho d)
figure(6)
plot(out.PLCrho1.Time,out.PLCrho1.Data(:,1),'k','LineWidth',1.5);
hold on

plot(out.LQRrho1.Time,out.LQRrho1.Data(:,1),'g','LineWidth',1.5);
hold on

plot(out.CNFrho1.Time,out.CNFrho1.Data(:,1),'b','LineWidth',1.5);
hold on

plot(out.CQRrho1.Time,out.CQRrho1.Data(:,1),'r-o','MarkerIndices',1:500:length(out.CQRstates.Data(1,:)),'LineWidth',1.5);
hold on


plot(out.FCrho1.Time,out.FCrho1.Data(:,1),'m-+','MarkerIndices',1:500:length(out.FCstates.Data(1,:)),'LineWidth',1.5);
hold on


xlabel('Time(s)','fontsize',20)
ylabel('$\sqrt{\rho}$','interpreter','latex','fontsize',20)
set(gca,'xtick',0:1:10,'color',[1,1,1],'LineWidth',1,'fontsize',20)
grid on
set(gcf,'Position',[300 300 720 530])
%set(gcf,'Position',[300 300 400 300])
legend('P-LQR','LQR','CNF','CQR','Direct-FC','interpreter','latex','NumColumns',1)
legend('boxon')