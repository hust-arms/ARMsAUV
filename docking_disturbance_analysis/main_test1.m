clc;
clear;
close all;
%% 测试艇体表面取点函数points_on_auv
len = 2.712; rad = 0.12; dx = len/50; dtheta = 5*3.14/180;
[x_auvb, y_auvb, z_auvb, num] = points_on_auv(len, rad, dx, dtheta);
%figure('Name','艇体表面点离散分布','NumberTitle','off')
%plot3(x_auvb,y_auvb,z_auvb,'Color','r','LineWidth',1);
scatter3(x_auvb, y_auvb, z_auvb, 'k');
xlabel('x[m]'); ylabel('y[m]');zlabel('z[m]');
set(gca,'xlim',[-1.5 1.5],'ylim',[-0.5 0.5],'zlim', [-0.5 0.5], 'Fontsize',15)      %设置坐标轴范围及字体大小
title('艇体表面点离散分布','Fontsize',10)