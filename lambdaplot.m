clc;
clear;

jointspace = [537.5207943,502.9393499,479.4401706,416.3495018,490.5246273];
endeffector = [248.8784683,277.5283553,322.9072754,338.0758394,389.9427873];
time = [54.4393,18.0665,7.26291,4.73874,3.62143];
lambda = [0.1,0.3,0.5,0.7,0.9];
plot(lambda,jointspace,'r-o','LineWidth',2,'MarkerSize',8,'MarkerFaceColor','r');
hold on
plot(lambda,endeffector,'b-s','LineWidth',2,'MarkerSize',8,'MarkerFaceColor','b');
hold on
plot(lambda,time,'-^','LineWidth',2,'MarkerSize',8,'MarkerFaceColor','#EDB120');
set(gca,'fontsize',20);
title(['\fontname{times new roman}','Compare distance and time with \lambda value'],'fontsize',35);
xlabel(['\fontname{times new roman}','\lambda value'],'fontsize',20);
h_legend = legend({'joint distance','endeffector distance','time(sec)'});
set(h_legend,'fontname','times new roman','fontsize',15);