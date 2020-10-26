function [deltar,deltab,deltas] = control(ROB,REF)
%% ȫ�ֱ�������
%%ͧ�屾�����
global m W B x_B y_B z_B x_G y_G z_G I_xx I_yy I_zz;
%%ˮ��������
global X_dotu Y_dotv Y_dotr Z_dotw Z_dotq K_dotp M_dotw M_dotq N_dotv N_dotr;
global X_uu Y_vv Y_rr Z_ww Z_qq K_pp M_ww M_qq N_vv N_rr;
global Y_uv Y_ur Z_uw Z_uq M_uw M_uq N_uv N_ur;
%%����������
global Y_uudr Z_uuds Z_uudb M_uuds M_uudb N_uudr;
%���Ʋ���
global c_z k_z alpha_z 
global c_theta k_theta alpha_theta
global c_psi k_psi alpha_psi
global boundary_thick

%% ͧ��״̬��Ϣ��ȡ
x = ROB(1);       y = ROB(2);        z = ROB(3);  
phi = ROB(4);     theta = ROB(5);    psi = ROB(6);  
u = ROB(7);       v = ROB(8);        w = ROB(9);  
p = ROB(10);      q = ROB(11);       r = ROB(12);
%% ������Ϣ��ȡ
refz=REF(1);       refdot_z=REF(2);       refdot2_z=REF(3);   
reftheta=REF(4);   refdot_theta=REF(5);   refdot2_theta=REF(6);   
refpsi=REF(7);     refdot_psi=REF(8);     refdot2_psi=REF(9);
%% �����ͧ��״̬����
%�����˶����̱����滻
a_zw = m-Z_dotw;     a_zq = -(m*x_G+Z_dotq);
a_zs = Z_uuds*u*u;   a_zb = Z_uudb*u*u;
f_z = m*u*q + m*z_G*q*q - X_dotu*u*q + Z_ww*w*abs(w) + Z_uw*u*w + Z_qq*q*abs(q) + Z_uq*u*q + (W-B)*cos(theta);
%�����˶����̱����滻(�����е�tΪtheta����д)
a_tw = -(m*x_G+M_dotw);  a_tq = I_yy - M_dotq;
a_ts = M_uuds*u*u;       a_tb = M_uudb*u*u;
f_t = -m*z_G*w*q - m*x_G*u*q - (Z_dotw*w+Z_dotq*q)*u + X_dotu*u*w + M_ww*w*abs(w) + M_uw*u*w + M_qq*q*abs(q)...
    +M_uq*u*q - (z_G*W-z_B*B)*sin(theta) - (x_G*W-x_B*B)*cos(theta);
%dot_w��dot_q���ʽ�е���
b_z = (a_tq*f_z-a_zq*f_t)/(a_zw*a_tq-a_zq*a_tw);
b_zb = (a_tq*a_zb-a_zq*a_tb)/(a_zw*a_tq-a_zq*a_tw);
b_zs = (a_tq*a_zs-a_zq*a_ts)/(a_zw*a_tq-a_zq*a_tw);
b_t = (a_zw*f_t-a_tw*f_z)/(a_zw*a_tq-a_zq*a_tw);
b_tb = (a_zw*a_tb-a_tw*a_zb)/(a_zw*a_tq-a_zq*a_tw);
b_ts = (a_zw*a_ts-a_tw*a_zs)/(a_zw*a_tq-a_zq*a_tw);
%dot2_z��dot2_theta���ʽ�е���
g_z = b_z*cos(theta) - u*q*cos(theta) - w*q*sin(theta);
g_zb = b_zb*cos(theta);   g_zs = b_zs*cos(theta);
g_t = b_t;
g_tb = b_tb;   g_ts = b_ts;
%��ȼ������һ�׵�
dot_z = -u*sin(theta) + w*cos(theta);
dot_theta = q;

%% ˮƽ��ͧ��״̬����
%�����˶����̱����滻(�����е�dΪdelta����д,pΪpsi����д)
a_yv = m-Y_dotv;  a_yr = m*x_G-Y_dotr;  a_ydr = Y_uudr*u*u;
f_y = m*y_G*r*r - m*u*r + X_dotu*u*r + Y_vv*v*abs(v) + Y_uv*u*v + Y_rr*r*abs(r) + Y_ur*u*r;
%��ҡ�˶����̱����滻
a_pv = m*x_G-N_dotv;  a_pr = I_zz-N_dotr;  a_pdr = N_uudr*u*u;
f_p = -m*x_G*u*r - m*y_G*v*r + (Y_dotv*v+Y_dotr*r)*u - X_dotu*u*v + N_vv*v*abs(v) + N_uv*u*v + N_rr*r*abs(r) + N_ur*u*r;
%dot_v��dot_r���ʽ�е���
b_y = (a_pr*f_y-a_yr*f_p)/(a_yv*a_pr-a_pv*a_yr);
b_ydr = (a_pr*a_ydr-a_yr*a_pdr)/(a_yv*a_pr-a_pv*a_yr);
b_p = (a_yv*f_p-a_pv*f_y)/(a_yv*a_pr-a_pv*a_yr);
b_pdr = (a_yv*a_pdr-a_pv*a_ydr)/(a_yv*a_pr-a_pv*a_yr);
%����psi��һ�׵�
dot_psi = r; 

%% ��ģ����
%���
e_z = z - refz;
dot_e_z = dot_z - refdot_z;
S_z = dot_e_z + c_z*e_z;

%����
e_theta = theta - reftheta;
dot_e_theta = dot_theta - refdot_theta;
S_theta = dot_e_theta + c_theta*e_theta;

%����
e_psi = psi - refpsi;
dot_e_psi = dot_psi - refdot_psi;
S_psi = dot_e_psi + c_psi*e_psi;

L_z = refdot2_z - g_z - c_z*dot_e_z - k_z*power(abs(S_z),alpha_z)*sat(S_z,boundary_thick);
L_theta = refdot2_theta - g_t - c_theta*dot_e_theta - k_theta*power(abs(S_theta),alpha_theta)*sat(S_theta,boundary_thick);
L_psi = refdot2_psi - b_p - c_psi*dot_e_psi - k_psi*power(abs(S_psi),alpha_psi)*sat(S_psi,boundary_thick);

%ָ�����
deltab = (L_z*g_ts - L_theta*g_zs)/(g_zb*g_ts - g_tb*g_zs);
deltas = (L_theta*g_zb - L_z*g_tb)/(g_zb*g_ts - g_tb*g_zs);
deltar = L_psi/b_pdr;
end