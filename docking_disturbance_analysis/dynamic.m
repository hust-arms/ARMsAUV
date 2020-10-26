function [ROBNew] = dynamic(ROB,deltar,deltab,deltas)
%% 全局变量声明
%%艇体本体参数
global m W B x_B y_B z_B x_G y_G z_G I_xx I_yy I_zz;
%%水动力参数
global X_dotu Y_dotv Y_dotr Z_dotw Z_dotq K_dotp M_dotw M_dotq N_dotv N_dotr;
global X_uu Y_vv Y_rr Z_ww Z_qq K_pp M_ww M_qq N_vv N_rr;
global Y_uv Y_ur Z_uw Z_uq M_uw M_uq N_uv N_ur;
%%控制力参数
global Y_uudr Z_uuds Z_uudb M_uuds M_uudb N_uudr;
global u_const dt;
%% 艇体状态信息提取
x = ROB(1);       y = ROB(2);        z = ROB(3);  
phi = ROB(4);     theta = ROB(5);    psi = ROB(6);  
u = ROB(7);       v = ROB(8);        w = ROB(9);  
p = ROB(10);      q = ROB(11);       r = ROB(12);

%% 艇体状态计算
%垂向运动方程变量替换
a_zw = m-Z_dotw;     a_zq = -(m*x_G+Z_dotq);
a_zs = Z_uuds*u*u;   a_zb = Z_uudb*u*u;
f_z = m*u*q + m*z_G*q*q - X_dotu*u*q + Z_ww*w*abs(w) + Z_uw*u*w + Z_qq*q*abs(q) + Z_uq*u*q + (W-B)*cos(theta);
%纵倾运动方程变量替换(变量中的t为theta的缩写)
a_tw = -(m*x_G+M_dotw);  a_tq = I_yy - M_dotq;
a_ts = M_uuds*u*u;       a_tb = M_uudb*u*u;
f_t = -m*z_G*w*q - m*x_G*u*q - (Z_dotw*w+Z_dotq*q)*u + X_dotu*u*w + M_ww*w*abs(w) + M_uw*u*w + M_qq*q*abs(q)...
    +M_uq*u*q - (z_G*W-z_B*B)*sin(theta) - (x_G*W-x_B*B)*cos(theta);

b_z = (a_tq*f_z-a_zq*f_t)/(a_zw*a_tq-a_zq*a_tw);
b_zb = (a_tq*a_zb-a_zq*a_tb)/(a_zw*a_tq-a_zq*a_tw);
b_zs = (a_tq*a_zs-a_zq*a_ts)/(a_zw*a_tq-a_zq*a_tw);
b_t = (a_zw*f_t-a_tw*f_z)/(a_zw*a_tq-a_zq*a_tw);
b_tb = (a_zw*a_tb-a_tw*a_zb)/(a_zw*a_tq-a_zq*a_tw);
b_ts = (a_zw*a_ts-a_tw*a_zs)/(a_zw*a_tq-a_zq*a_tw);

%横向运动方程变量替换(变量中的d为delta的缩写,p为psi的缩写)
a_yv = m-Y_dotv;  a_yr = m*x_G-Y_dotr;  a_ydr = Y_uudr*u*u;
f_y = m*y_G*r*r - m*u*r + X_dotu*u*r + Y_vv*v*abs(v) + Y_uv*u*v + Y_rr*r*abs(r) + Y_ur*u*r;
%横摇运动方程变量替换
a_pv = m*x_G-N_dotv;  a_pr = I_zz-N_dotr;  a_pdr = N_uudr*u*u;
f_p = -m*x_G*u*r - m*y_G*v*r + (Y_dotv*v+Y_dotr*r)*u - X_dotu*u*v + N_vv*v*abs(v) + N_uv*u*v + N_rr*r*abs(r) + N_ur*u*r;

b_y = (a_pr*f_y-a_yr*f_p)/(a_yv*a_pr-a_pv*a_yr);
b_ydr = (a_pr*a_ydr-a_yr*a_pdr)/(a_yv*a_pr-a_pv*a_yr);
b_p = (a_yv*f_p-a_pv*f_y)/(a_yv*a_pr-a_pv*a_yr);
b_pdr = (a_yv*a_pdr-a_pv*a_ydr)/(a_yv*a_pr-a_pv*a_yr);

%% 艇体状态更新
ROBdot_x = u*cos(psi)*cos(theta) - v*sin(psi) + w*cos(psi)*sin(theta);
ROBdot_y = u*sin(psi) + v*cos(psi);
ROBdot_z = -u*sin(theta) + w*cos(theta);
ROBdot_phi = 0;
ROBdot_theta = q;
ROBdot_psi = r; 
ROBdot_u = 0;
ROBdot_v = b_y + b_ydr*deltar; 
ROBdot_w = b_z + b_zb*deltab + b_zs*deltas; 
ROBdot_p = 0; 
ROBdot_q = b_t + b_tb*deltab + b_ts*deltas; 
ROBdot_r = b_p + b_pdr*deltar;  

x = x + ROBdot_x*dt;
y = y + ROBdot_y*dt;
z = z + ROBdot_z*dt;
phi = 0;
theta = theta + ROBdot_theta*dt;
psi = psi + ROBdot_psi*dt;
u = u_const;
v = v + ROBdot_v*dt;
w = w + ROBdot_w*dt;
p = 0;
q = q + ROBdot_q*dt;
r = r + ROBdot_r*dt;
ROBNew = [x,y,z,phi,theta,psi,u,v,w,p,q,r];
end