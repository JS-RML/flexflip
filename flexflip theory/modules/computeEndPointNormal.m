function [end_normal_u, end_normal_v] = computeEndPointNormal(var_theta)
    %{ 
    Return components of normal vector to the curve at its end-point. 
    %}

    if var_theta(end)<=0
        end_normal_u = -sin(var_theta(end));
        end_normal_v = cos(var_theta(end));
    else
        end_normal_u = sin(var_theta(end));
        end_normal_v = -cos(var_theta(end));
    end

end

