function [x_auv, y_auv, z_auv] = auv_to_world(x_auvb, y_auvb, z_auvb, x_w, y_w, z_w, phi, theta, psi, num)
%% Transform auv coordinate into world frame
x_auv = []; y_auv = []; z_auv = [];
for k=1:num
x_auv(k) = x_w + x_auvb(k) * cos(psi) * cos(theta) + y_auvb(k) * (cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi)) + z_auvb(k) * (cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi));
y_auv(k) = y_w + x_auvb(k) * sin(psi) * cos(theta) + y_auvb(k) * (sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi)) + z_auvb(k) * (sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi));
z_auv(k) = z_w - x_auvb(k) * sin(theta) + y_auvb(k) * cos(theta) * sin(phi) + z_auvb(k) * cos(theta) * cos(phi);
end
end
