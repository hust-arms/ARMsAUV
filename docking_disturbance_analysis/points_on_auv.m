function [x_auvb, y_auvb, z_auvb, num] = points_on_auv(len, rad, dx, dtheta)
x=-len/2; theta=0;
x_auvb = []; y_auvb = []; z_auvb = [];
num = 2;
x_auvb(1) = x;
y_auvb(1) = rad;
z_auvb(1) = 0;
if dx < 0
    return;
end
theta = theta + dtheta;
while x <= len/2
    while theta <= 2*3.14
        x_auvb(num) = x;
        y_auvb(num) = rad * cos(theta);
        z_auvb(num) = rad * sin(theta);
        num = num + 1;
        theta = theta + dtheta;
    end
    x = x + dx;
    theta=0;
end
num = num - 1;
end