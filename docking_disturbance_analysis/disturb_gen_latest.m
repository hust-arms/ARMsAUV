function [a_xdamp, a_ydamp, a_zdamp, is_qt_disturb] = disturb_gen_latest(x_qt, y_qt, z_qt, x_auv, y_auv, z_auv, L_qt, W_qt, H_qt, t)
%% β�������Ŷ����ǿ��
a_m = 0.05;
%% ���������Ŷ����ǿ��
a_m_o_fixed = 0;
a_m_o_sin = 0.0005;
a_m_o_rand = 0.001;
%% �Ŷ����÷�Χ
x0 = 30;
y0 = 6;
z0 = 6;
dtheshold = sqrt((x0+L_qt/2) * (x0+L_qt/2) + (y0+W_qt/2) * (y0+W_qt/2) + (z0+H_qt/2) * (z0+H_qt/2));
%% ����β��x��y��z�����Ŷ�(-a_m,a_m)
%a_x = a_m * (rand(1)*(-2)+1);
%a_y = a_m * (rand(1)*(-2)+1);
%a_z = a_m * (rand(1)*(-2)+1);
a_x = a_m;
a_y = a_m;
a_z = a_m;
is_qt_disturb = 1;
%% ����Ǳͧβ����AUVĳ����������Ŷ�ǿ��
dx_mah = abs(x_qt - x_auv);
dy_mah = abs(y_qt - y_auv);
dz_mah = abs(z_qt - z_auv);
%dx_euc = abs(sqrt(dx_mah * dx_mah)-L_qt/2);
%dy_euc = abs(sqrt(dy_mah * dy_mah)-W_qt/2);
%dz_euc = abs(sqrt(dz_mah * dz_mah)-H_qt/2);
dx_euc = sqrt(dx_mah * dx_mah);
dy_euc = sqrt(dy_mah * dy_mah);
dz_euc = sqrt(dz_mah * dz_mah);
dist = sqrt(dx_mah * dx_mah + dy_mah * dy_mah + dz_mah * dz_mah);
if dist > dtheshold
    %a_xdamp = 0;
    %% ���ﲻ��β���Ŷ����÷�Χ�������������
    a_xdamp = a_m_o_fixed + a_m_o_sin * sin(t) + a_m_o_rand * (rand(1)*(-2)+1);
    %a_xdamp = a_m_o_fixed + a_m_o_rand * (rand(1)*(-2)+1);
    is_qt_disturb = 0;
else
    dx = x_qt - x_auv;
    if  dx >= L_qt/2 && dx <= L_qt/2 + x0 % x�������÷�Χ(-x0, -L/2)
        % a_xdamp = a_x * (1 - 1/(1 + exp(-dx_euc) + x0 / 2));
        a_xdamp = -a_x * (1 - 1/(1 + exp(-dx_euc + x0 / 2)));
    else
        a_xdamp = 0;
    end
end
if dist > dtheshold
    %a_ydamp = 0;
    %% ���ﲻ��β���Ŷ����÷�Χ�������������
    a_ydamp = a_m_o_fixed + a_m_o_sin * sin(t) + a_m_o_rand * (rand(1)*(-2)+1);
    %a_ydamp = a_m_o_fixed + a_m_o_rand * (rand(1)*(-2)+1);
    is_qt_disturb = 0;
else
    %if dist < y0 || dist == y0
        dy = y_qt - y_auv;
        if(dy >= W_qt/2) && (dy <= W_qt/2 + y0)
            a_ydamp = a_y * (1 - 1/(1 + exp(-dy_euc + y0 / 2)));
        elseif(dy <= -W_qt/2) && (dy >= -W_qt/2 - y0)
            a_ydamp = -a_y * (1 - 1/(1 + exp(-dy_euc + y0 / 2)));
        else
            a_ydamp = 0;
        end 
    %else
    %    a_ydamp = 0;
    %end
end
if dist > dtheshold
    %a_zdamp = 0;
    %% ���ﲻ��β���Ŷ����÷�Χ�������������
    a_zdamp = a_m_o_fixed + a_m_o_sin * sin(t) + a_m_o_rand * (rand(1)*(-2)+1);
    %a_zdamp = a_m_o_fixed + a_m_o_rand * (rand(1)*(-2)+1);
    is_qt_disturb = 0;
else
    %if dist < z0 || dist == z0
            dz = z_qt - z_auv;
        if(dz >= H_qt/2) && (dz <= H_qt/2 + z0)
            a_zdamp = a_z * (1 - 1/(1 + exp(-dz_euc + z0 / 2)));
        elseif(dz <= -H_qt/2) && (dz >= -H_qt/2 - z0)
            a_zdamp = -a_z * (1 - 1/(1 + exp(-dz_euc + z0 / 2)));
        else
            a_zdamp = 0;
        end 
    %else
    %    a_zdamp = 0;
    %end
end

end %% end of function