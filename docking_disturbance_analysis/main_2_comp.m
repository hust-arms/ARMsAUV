%���빦�ܣ�ģ���Ŷ���̬��Ӧ����
clc;
clear;
close all;
%% ȫ�ֱ�������
%%ͧ�屾�����
global m L W B x_B y_B z_B x_G y_G z_G I_xx I_yy I_zz rad;
m = 84.71;   L = 2.712;   %����/����
W = 831;  B = 838; %�����͸���,ʵ��ģ����B=838
x_B = 0; y_B = 0; z_B = 0; %��������
x_G = 0; y_G = 0; z_G = 0.0086; %��������
I_xx = 0.82; I_yy = 30.14; I_zz = 30.14;  %���Ծ�
rad = 0.12; %�뾶

%%ˮ��������
global X_dotu Y_dotv Y_dotr Z_dotw Z_dotq K_dotp M_dotw M_dotq N_dotv N_dotr;
global X_uu Y_vv Y_rr Z_ww Z_qq K_pp M_ww M_qq N_vv N_rr;
global Y_uv Y_ur Z_uw Z_uq M_uw M_uq N_uv N_ur;
X_dotu = -1.432; Y_dotv = -120.645; Y_dotr = -4.895; Z_dotw = -130.513; Z_dotq = 16.488; K_dotp = -0.386;
M_dotw = 16.488; M_dotq = -78.266; N_dotv = -4.895; N_dotr = -67.489;
X_uu = -3.9; Y_vv = -373.287; Y_rr = -4.204; Z_ww = -489.07; Z_qq = 23.016; K_pp = -0.1177; M_ww = 23.342;
M_qq = -353.406; N_vv = 0.4193; N_rr = -227.024;
Y_uv = -130.64; Y_ur = 40.25; Z_uw = -522.87; Z_uq = 4.27; M_uw = 140.68; M_uq = 73; N_uv = -57.47; N_ur = -50.3;

%%����������
global Y_uudr Z_uuds Z_uudb M_uuds M_uudb N_uudr;
Y_uudr = 38.279; Z_uuds = -38.279; Z_uudb = -44.981; M_uuds = 41.686; M_uudb = -44.531; N_uudr = -41.686;
%%���Ʋ���
global  dt  tfinal  t k u_const  z_d  psi_d x_d y_d;
dt=0.1; tfinal=150; t=0; k=1;  %%����/�ܷ���ʱ��/����ʱ��/����������
%% ����Ŀ����
u_const=3*0.514;  z_d = 0;   
% psi_d=30/57.3;  %%����Ϊ6��/�������/��������
psi_d = 0;
% x_d = 30; y_d = 0;   %��������ؾ���
x_d = 0; y_d = 0;
N = tfinal/dt+1;
%%���Ʋ���
global c_z k_z alpha_z 
global c_theta k_theta alpha_theta
global c_psi k_psi alpha_psi
global boundary_thick
c_z = 0.1; k_z = 0.2; alpha_z = 0.8; 
c_theta = 0.2; k_theta = 0.25; alpha_theta = 0.8;
c_psi = 0.1; k_psi = 0.2; alpha_psi = 0.6;
boundary_thick = 0.1;
%%���鶨��
%����ģ�ͱ���
ROBx=zeros(1,N);       ROBy=zeros(1,N);        ROBz=zeros(1,N);
ROBphi=zeros(1,N);     ROBtheta=zeros(1,N);    ROBpsi=zeros(1,N);  %λ�ú���̬
ROBu=zeros(1,N);       ROBv=zeros(1,N);        ROBw=zeros(1,N);
ROBp=zeros(1,N);       ROBq=zeros(1,N);        ROBr=zeros(1,N);    %�ٶȺͽ��ٶ�
delta_b = zeros(1,N);  delta_s = zeros(1,N);   delta_r = zeros(1,N);      %������/������/�������
time = zeros(1,N);   %ʱ������

%���������������
REFz=zeros(1,N);       REFdot_z=zeros(1,N);       REFdot2_z=zeros(1,N);      %������ȼ���һ�׺Ͷ��׵���
REFtheta=zeros(1,N);   REFdot_theta=zeros(1,N);   REFdot2_theta=zeros(1,N);    %�������㼰��һ�׺Ͷ��׵���
REFpsi=zeros(1,N);     REFdot_psi=zeros(1,N);     REFdot2_psi=zeros(1,N);     %����������һ�׺Ͷ��׵���
ERRz=zeros(1,N);       ERRtheta=zeros(1,N);       ERRpsi=zeros(1,N);         %������/�������/�������
DIS = zeros(1,N);  %ƫ��
%% �����ʼ��
%ģ�ͱ�����ʼ��
ROBx(1)=0;        ROBy(1)=0;         ROBz(1)=0;     ROBphi(1)=0;      ROBtheta(1)=0;     ROBpsi(1)=0;
ROBu(1)=u_const;  ROBv(1)=0;         ROBw(1)=0;      ROBp(1)=0;        ROBq(1)=0;         ROBr(1)=0;
delta_b(1) = 0;   delta_s(1) = 0;    delta_r(1) = 0;
%����������ʼ��
REFz(1)=z_d;       REFdot_z(1)=0;       REFdot2_z(1)=0;   
REFtheta(1)=-1/3*atan(-(ROBz(1)-REFz(1))/(2*L)); REFdot_theta(1)=0; REFdot2_theta(1)=0;    
DIS(1) = (ROBx(1) - x_d)*sin(psi_d) - (ROBy(1) - y_d)*cos(psi_d);
REFpsi(1)=psi_d + atan(DIS(1)/10);   REFdot_psi(1)=0;     REFdot2_psi(1)=0;
ERRz(1)=ROBz(1)-REFz(1);     ERRtheta(1)=ROBtheta(1)-REFtheta(1);      ERRpsi(1)=ROBpsi(1)-REFpsi(1);   

%% Ǳͧλ��
QTx=zeros(1,N); QTy=zeros(1,N); QTz=zeros(1,N);
QTx(1)=40; QTy(1)=-2.5; QTz(1)=-1.0;
%x_qt=150; y_qt=1.5; z_qt=0;
u_const_qt=2*0.514;

%% ����ͧ����������
dtheta_sec=5*3.14/180;
dxauv=L/50;
[x_auvb, y_auvb, z_auvb, num] = points_on_auv(L, rad, dxauv, dtheta_sec);

tau_dx_v = zeros(1,N); tau_dy_v = zeros(1,N); tau_dz_v = zeros(1,N); tau_dphi_v = zeros(1,N); tau_dtheta_v = zeros(1,N); tau_dpsi_v = zeros(1,N);

%% Ǳͧ-AUVλ�ò�
DeltaX = zeros(1,N+1); DeltaY = zeros(1,N+1); DeltaZ = zeros(1,N+1);

%% AUV�˶�ѭ��
while t < tfinal
    is_qt_disturb = 1;
    ROB = [ROBx(k),ROBy(k),ROBz(k),ROBphi(k),ROBtheta(k),ROBpsi(k),ROBu(k),ROBv(k),ROBw(k),ROBp(k),ROBq(k),ROBr(k)];
    REF = [REFz(k),REFdot_z(k),REFdot2_z(k),REFtheta(k),REFdot_theta(k),REFdot2_theta(k),REFpsi(k),REFdot_psi(k),REFdot2_psi(k)];
    [deltar,deltab,deltas] = control(ROB,REF);
    %deltar=0; deltab=0; deltas=0;
    [tau_dx, tau_dy, tau_dz, tau_dphi, tau_dtheta, tau_dpsi, is_qt_disturb] = disturb_cal_comp(QTx(k), QTy(k), QTz(k), x_auvb, y_auvb, z_auvb,  ROBx(k), ROBy(k), ROBz(k), num, ROBphi(k), ROBtheta(k), ROBpsi(k), t);
    
    %% ����AUV��Ǳͧ�����Ŷ���
    %if(QTy(k)>ROBy(k) && is_qt_disturb)
    %   tau_dy = -abs(tau_dy);
    %end
    %if(QTz(k)>ROBz(k) && is_qt_disturb)
    %   tau_dz = -abs(tau_dz);
    %end
    
    
    %% ����λ�þ���
    DeltaX(k) = QTx(k) - ROBx(k); DeltaY(k) = QTy(k) - ROBy(k); DeltaZ(k) = QTz(k) - ROBz(k); 
    
    [ROBNew] = disturbance_dynamic(ROB,deltar,deltab,deltas,tau_dx, tau_dy, tau_dz, tau_dphi, tau_dtheta, tau_dpsi); %����AUV״̬
    %[ROBNew] = dynamic(ROB,deltar,deltab,deltas); %����AUV״̬
    k = k+1;
    QTx(k)=QTx(k-1)+u_const_qt * dt; QTy(k)=QTy(k-1); QTz(k)=QTz(k-1);
    
    tau_dx_v(k) = tau_dx; tau_dy_v(k) = tau_dy; tau_dz_v(k) = tau_dz; tau_dphi_v(k) = tau_dphi; tau_dtheta_v(k) = tau_dtheta; tau_dpsi_v(k) = tau_dpsi;
    
    delta_b(k) = deltab;  delta_s(k) = deltas;   delta_r(k) = deltar;
    ROBx(k)=ROBNew(1);        ROBy(k)=ROBNew(2);         ROBz(k)=ROBNew(3);
    ROBphi(k)=ROBNew(4);      ROBtheta(k)=ROBNew(5);     ROBpsi(k)=ROBNew(6);
    ROBu(k)=ROBNew(7);        ROBv(k)=ROBNew(8);         ROBw(k)=ROBNew(9);
    ROBp(k)=ROBNew(10);       ROBq(k)=ROBNew(11);        ROBr(k)=ROBNew(12);

    t = t+dt;
    time(k) = t;
end
%%  �ռ�λ������
figure('Name','�ռ�λ������','NumberTitle','off');
plot3(ROBx,ROBy,ROBz, 'r','Linewidth',1.5);
hold on
plot3(QTx, QTy, QTz, '.', 'Color','b', 'MarkerSize', 10.0);
xlabel('x[m]'); ylabel('y[m]'); zlabel('z[m]');
set(gca,'xlimmode','auto','ylimmode','auto','zlim',[-10 10],'Fontsize',15)      %���������᷶Χ�������С
title('�ռ�λ������','Fontsize',20)
grid;

%%  �ռ�λ�ò�����
figure('Name','λ�ò�����','NumberTitle','off')
subplot(3,1,1)
numv = 1:1:num;
plot(time(1:N), DeltaX(1:N), 'Color','r','Linewidth',1.5);
xlabel('time[s]'); ylabel('x[m]');
set(gca,'xlimmode','auto','ylimmode','auto','Fontsize',15);    %���������᷶Χ�������С
title('x�����','Fontsize',20);
subplot(3,1,2);
plot(time(1:N), DeltaY(1:N), 'Color','r','Linewidth',1.5);
xlabel('time[s]');   ylabel('y[m]');
set(gca,'xlimmode','auto','ylimmode','auto','Fontsize',15);
title('y�����','Fontsize',20);
subplot(3,1,3);
plot(time(1:N), DeltaZ(1:N), 'Color','r','Linewidth',1.5);
xlabel('time[s]');   ylabel('z[m]');
set(gca,'xlimmode','auto','ylimmode','auto','Fontsize',15);
title('z�����','Fontsize',20);

%% ��������Ŷ�����
figure('Name','�����Ŷ�����','NumberTitle','off')
subplot(3,1,1)
numv = 1:1:num;
plot(time, tau_dx_v, 'Color','r','Linewidth',1.5);
xlabel('time[s]'); ylabel('taux');
set(gca,'xlimmode','auto','ylimmode','auto','Fontsize',15);    %���������᷶Χ�������С
title('x���Ŷ�','Fontsize',20);
subplot(3,1,2);
plot(time, tau_dy_v, 'Color','r','Linewidth',1.5);
xlabel('time[s]');   ylabel('tauy');
set(gca,'xlimmode','auto','ylimmode','auto','Fontsize',15);
title('y���Ŷ�','Fontsize',20);
subplot(3,1,3);
plot(time, tau_dz_v, 'Color','r','Linewidth',1.5);
xlabel('time[s]');   ylabel('tauz');
set(gca,'xlimmode','auto','ylimmode','auto','Fontsize',15);
title('z���Ŷ�','Fontsize',20);

%% ����������Ŷ�����
figure('Name','�������Ŷ�����','NumberTitle','off')
subplot(3,1,1);
plot(time, tau_dphi_v, 'Color','r','Linewidth',1.5);
xlabel('time[s]');   ylabel('tauphi');
set(gca,'xlimmode','auto','ylimmode','auto','Fontsize',15);
title('����Ŷ�','Fontsize',20);
subplot(3,1,2);
plot(time, tau_dtheta_v, 'Color','r','Linewidth',1.5);
xlabel('time[s]');   ylabel('tautheta');
set(gca,'xlimmode','auto','ylimmode','auto','Fontsize',15);
title('�����Ŷ�','Fontsize',20);
subplot(3,1,3);
plot(time, tau_dpsi_v, 'Color','r','Linewidth',1.5);
xlabel('time[s]');   ylabel('taupsi');
set(gca,'xlimmode','auto','ylimmode','auto','Fontsize',15);
title('�����Ŷ�','Fontsize',20);


%%  ˮƽ��λ������
% figure('Name','ˮƽ��λ������','NumberTitle','off')
% plot(ROBx,ROBy, 'r','Linewidth',1.5)
% axis equal
% xlabel('x[m]'); ylabel('y[m]');
% % set(gca,'xlimmode','auto','ylimmode','auto','zlimmode','auto','Fontsize',15)      %���������᷶Χ�������С
% title('ˮƽ��λ������','Fontsize',20)

%%  ��ֱ��λ������
% figure('Name','��ֱ��λ������','NumberTitle','off')
% plot(ROBx,ROBz, 'r','Linewidth',1.5)
% xlabel('x[m]'); ylabel('z[m]');
% % set(gca,'xlimmode','auto','ylimmode','auto','zlimmode','auto','Fontsize',15)      %���������᷶Χ�������С
% title('��ֱ��λ������','Fontsize',20)

%%  ƫ������
% figure('Name','ƫ������','NumberTitle','off')
% plot(time,DIS, 'r','Linewidth',1.5)
% xlabel('time[m]'); ylabel('DIS[m]');
% set(gca,'xlim',[0 tfinal],'ylimmode','auto','zlimmode','auto','Fontsize',15)      %���������᷶Χ�������С
% title('ƫ������','Fontsize',20)
%%  ����λ������
% figure('Name','����λ������','NumberTitle','off')
% subplot(3,1,1)
% plot(time,ROBx, 'r','Linewidth',1.5)
% xlabel('time[s]'); ylabel('x[m]');
% set(gca,'xlim',[0 tfinal],'ylimmode','auto','Fontsize',15)      %���������᷶Χ�������С
% title('x�������','Fontsize',20)
% subplot(3,1,2)
% plot(time, ROBy, 'r','Linewidth',1.5)
% xlabel('time[s]');   ylabel('y[m]');
% set(gca,'xlim',[0 tfinal],'ylimmode','auto','Fontsize',15);
% title('y�������','Fontsize',20)
% subplot(3,1,3)
% plot(time, ROBz, 'r','Linewidth',1.5)
% xlabel('time[s]');   ylabel('z[m]');
% set(gca,'xlim',[0 tfinal],'ylimmode','auto','Fontsize',15);
% title('z�������','Fontsize',20)

%%  ��̬����
figure('Name','��̬����','NumberTitle','off')
subplot(3,1,1)
plot(time,57.3*ROBphi, 'r','Linewidth',1.5)
xlabel('time[s]'); ylabel('phi[\circ]');
set(gca,'xlim',[0 tfinal],'ylimmode','auto','Fontsize',15)      %���������᷶Χ�������С
title('�����','Fontsize',20)
subplot(3,1,2)
plot(time, 57.3*ROBtheta, 'r','Linewidth',1.5)
xlabel('time[s]');   ylabel('theta[\circ]');
set(gca,'xlim',[0 tfinal],'ylimmode','auto','Fontsize',15);
title('�����','Fontsize',20)
subplot(3,1,3)
plot(time, 57.3*ROBpsi, 'r','Linewidth',1.5)
xlabel('time[s]');   ylabel('psi[\circ]');
set(gca,'xlim',[0 tfinal],'ylimmode','auto','Fontsize',15);
title('�����','Fontsize',20)

%%  �ٶ�����
figure('Name','�ٶ�����','NumberTitle','off')
subplot(3,1,1)
plot(time,ROBu, 'r','Linewidth',1.5)
xlabel('time[s]'); ylabel('u[m/s]');
set(gca,'xlim',[0 tfinal],'ylimmode','auto','Fontsize',15)      %���������᷶Χ�������С
title('�����ٶ�','Fontsize',20)
subplot(3,1,2)
plot(time, ROBv, 'r','Linewidth',1.5)
xlabel('time[s]');   ylabel('v[m/s]');
set(gca,'xlim',[0 tfinal],'ylimmode','auto','Fontsize',15);
title('�����ٶ�','Fontsize',20)
subplot(3,1,3)
plot(time, ROBw, 'r','Linewidth',1.5)
xlabel('time[s]');   ylabel('w[m/s]');
set(gca,'xlim',[0 tfinal],'ylimmode','auto','Fontsize',15);
title('�����ٶ�','Fontsize',20)

%%  ���ٶ�����
figure('Name','���ٶ�����','NumberTitle','off')
subplot(3,1,1)
plot(time,57.3*ROBp, 'r','Linewidth',1.5)
xlabel('time[s]'); ylabel('p[\circ/s]');
set(gca,'xlim',[0 tfinal],'ylimmode','auto','Fontsize',15)      %���������᷶Χ�������С
title('������ٶ�','Fontsize',20)
subplot(3,1,2)
plot(time, 57.3*ROBq, 'r','Linewidth',1.5)
xlabel('time[s]');   ylabel('q[\circ/s]');
set(gca,'xlim',[0 tfinal],'ylimmode','auto','Fontsize',15);
title('������ٶ�','Fontsize',20)
subplot(3,1,3)
plot(time, 57.3*ROBr, 'r','Linewidth',1.5)
xlabel('time[s]');   ylabel('r[\circ/s]');
set(gca,'xlim',[0 tfinal],'ylimmode','auto','Fontsize',15);
title('������ٶ�','Fontsize',20)

%%  �������
figure('Name','�������','NumberTitle','off')
subplot(3,1,1)
plot(time,57.3*delta_b, 'r','Linewidth',1.5)
xlabel('time[s]'); ylabel('delta_b[\circ/s]');
set(gca,'xlim',[0 tfinal],'ylimmode','auto','Fontsize',15)      %���������᷶Χ�������С
title('������','Fontsize',20)
subplot(3,1,2)
plot(time, 57.3*delta_s, 'r','Linewidth',1.5)
xlabel('time[s]');   ylabel('delta_s[\circ/s]');
set(gca,'xlim',[0 tfinal],'ylimmode','auto','Fontsize',15);
title('������','Fontsize',20)
subplot(3,1,3)
plot(time, 57.3*delta_r, 'r','Linewidth',1.5)
xlabel('time[s]');   ylabel('delta_r[\circ/s]');
set(gca,'xlim',[0 tfinal],'ylimmode','auto','Fontsize',15);
title('�������','Fontsize',20)

