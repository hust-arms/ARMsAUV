function [a_xdamp, a_ydamp, a_zdamp, is_qt_disturb] = disturb_gen(x_qt, y_qt, z_qt, x_auv, y_auv, z_auv)
%% β�������Ŷ����ǿ��
a_m = 0.005;
%% ���������Ŷ����ǿ��
a_m_o = 0.001;
%% �Ŷ�������þ���
x0 = 15;
y0 = 4;
z0 = 4;
dtheshold = sqrt(x0 * x0 + y0 * y0 + z0 * z0);
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
dx_euc = sqrt(dx_mah * dx_mah);
dy_euc = sqrt(dy_mah * dy_mah);
dz_euc = sqrt(dz_mah * dz_mah);
dist = sqrt(dx_mah * dx_mah + dy_mah * dy_mah + dz_mah * dz_mah);
if dist > dtheshold
    % a_xdamp = 0;
    %% ���ﲻ��β���Ŷ����÷�Χ�������������
    a_xdamp = a_m_o * (rand(1)*(-2)+1);
    is_qt_disturb = 0;
else
    if dist < x0 
        % a_xdamp = a_x * (1 - 1/(1 + exp(-dx_euc) + x0 / 2));
        a_xdamp = -a_x * (1 - 1/(1 + exp(-dx_euc + x0 / 2)));
    elseif dist == x0
        a_xdamp = -a_x * (1 - 1/(1 + exp(-dx_euc + x0 / 2)));
    else
        a_xdamp = 0;
    end
end
if dist > dtheshold
    % a_ydamp = 0;
    %% ���ﲻ��β���Ŷ����÷�Χ�������������
    a_ydamp = a_m_o * (rand(1)*(-2)+1);
    is_qt_disturb = 0;
else
    if dist < y0 || dist == y0
        if(y_qt > y_auv)
            a_ydamp = a_y * (1 - 1/(1 + exp(-dy_euc + y0 / 2)));
        elseif(y_qt < y_auv)
            a_ydamp = -a_y * (1 - 1/(1 + exp(-dy_euc + y0 / 2)));
        else
            a_ydamp = 0;
        end 
    else
        a_ydamp = 0;
    end
end
if dist > dtheshold
    % a_zdamp = 0;
    %% ���ﲻ��β���Ŷ����÷�Χ�������������
    a_zdamp = a_m_o * (rand(1)*(-2)+1);
    is_qt_disturb = 0;
else
    if dist < z0 || dist == z0
        if(z_qt > z_auv)
            a_zdamp = a_z * (1 - 1/(1 + exp(-dz_euc + z0 / 2)));
        elseif(z_qt < z_auv)
            a_zdamp = -a_z * (1 - 1/(1 + exp(-dz_euc + z0 / 2)));
        else
            a_zdamp = 0;
        end 
    else
        a_zdamp = 0;
    end
end

end %% end of function