function U_flex = computeFlexureEnergy(var_s, var_theta)
    %{
    This function computes flexural energy of a curve without factoring in the rigidity constraint. 
    INPUT:
    \theta(s)
    OUTPUT:
    flexure energy U_flex
    %}
    
    dthetads = gradient(var_theta)./gradient(var_s);
    
    U_flex = trapz(var_s, dthetads.^2);
    
end
