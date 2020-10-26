function [ output ] = sat( input,boundary_thick )
%   此处显示详细说明
if abs(input) >= boundary_thick
    output = sign(input);
else
    output = input/boundary_thick;
end

end

