clc;
clear;
close all;
%% ����ͧ�����ȡ�㺯��points_on_auv
len = 2.712; rad = 0.12; dx = len/50; dtheta = 5*3.14/180;
[x_auvb, y_auvb, z_auvb, num] = points_on_auv(len, rad, dx, dtheta);
%figure('Name','ͧ��������ɢ�ֲ�','NumberTitle','off')
%plot3(x_auvb,y_auvb,z_auvb,'Color','r','LineWidth',1);
scatter3(x_auvb, y_auvb, z_auvb, 'k');
xlabel('x[m]'); ylabel('y[m]');zlabel('z[m]');
set(gca,'xlim',[-1.5 1.5],'ylim',[-0.5 0.5],'zlim', [-0.5 0.5], 'Fontsize',15)      %���������᷶Χ�������С
title('ͧ��������ɢ�ֲ�','Fontsize',10)