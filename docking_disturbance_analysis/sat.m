function [ output ] = sat( input,boundary_thick )
%   �˴���ʾ��ϸ˵��
if abs(input) >= boundary_thick
    output = sign(input);
else
    output = input/boundary_thick;
end

end

