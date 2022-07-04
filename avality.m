clc;
clear;

k1 = [0.8,0.4,0.2,0,0];
k2 = [1,1,0.8,0.4,0.4];
k3 = [1,1,1,0.8,0.8]
lambda = [0.1,0.3,0.5,0.7,0.9];
plot(lambda,k1,'r-o','LineWidth',2,'MarkerSize',8,'MarkerFaceColor','r');
hold on
plot(lambda,k2,'b-s','LineWidth',2,'MarkerSize',8,'MarkerFaceColor','b');
hold on
plot(lambda,k3,'g-^','LineWidth',2,'MarkerSize',8,'MarkerFaceColor','g');
title(['\fontname{times new roman}','Compare availability with \lambda value'],'fontsize',100);
xlabel(['\fontname{times new roman}','\lambda value'],'fontsize',15);
ylabel(['\fontname{times new roman}','Availability(Prob)'],'fontsize',15);
h_legend = legend({'K = 50','K = 100','K = 150'});
set(h_legend,'fontname','times new roman','fontsize',15);
set(gca,'fontsize',20);
ylim([0,1.2])