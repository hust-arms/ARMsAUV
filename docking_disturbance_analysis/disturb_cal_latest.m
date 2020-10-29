function [taus_dx, taus_dy, taus_dz, taus_dphi, taus_dtheta, taus_dpsi, is_qt_disturb] = disturb_cal_latest(x_qt, y_qt, z_qt, x_auvb, y_auvb, z_auvb, x_w, y_w, z_w, num, phi, theta, psi, lqt, wqt, hqt,t)
%% 转换AUV艇体上坐标点到惯性坐标系
[x_auv, y_auv, z_auv] = auv_to_world(x_auvb, y_auvb, z_auvb, x_w, y_w, z_w, phi, theta, psi, num);
taus_dx=0; taus_dy=0; taus_dz=0; taus_dphi=0; taus_dtheta=0; taus_dpsi=0;
is_qt_disturb = 1;
%% 计算扰动作用
for k=1:num
[a_xdamp, a_ydamp, a_zdamp, is_qt_disturb] = disturb_gen_latest(x_qt, y_qt, z_qt, x_auv(k), y_auv(k), z_auv(k),lqt,wqt,hqt, t);
tau_dx = a_xdamp * cos(psi) * cos(theta) + a_ydamp * sin(psi) * cos(theta) - a_zdamp * sin(theta);
tau_dy = a_xdamp * (cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi)) + a_ydamp * (sin(psi) * sin(theta) * sin(phi) + cos(phi) * cos(psi)) + a_zdamp * cos(theta) * sin(phi);
tau_dz = a_xdamp * (cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)) + a_ydamp * (sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)) + a_zdamp * cos(theta) * cos(phi);
%tau_dphi = - (tau_dy * z_auvb(k) + tau_dz * y_auvb(k));
%tau_dtheta = -(tau_dx * z_auvb(k) + tau_dz * x_auvb(k));
%tau_dpsi = -(tau_dx * y_auvb(k) + tau_dy * x_auvb(k));
tau_dphi = tau_dy * z_auvb(k) - tau_dz * y_auvb(k);
tau_dtheta = -tau_dx * z_auvb(k) + tau_dz * x_auvb(k);
tau_dpsi = tau_dx * y_auvb(k) - tau_dy * x_auvb(k);
taus_dx = taus_dx + tau_dx;
taus_dy = taus_dy + tau_dy;
taus_dz = taus_dz + tau_dz;
taus_dphi = taus_dphi + tau_dphi;
taus_dtheta = taus_dtheta + tau_dtheta;
taus_dpsi = taus_dpsi + tau_dpsi;
end
end