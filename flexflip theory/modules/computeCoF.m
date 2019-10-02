function CoF = computeCoF(lambda, var_theta)
    %{
    Compute coefficient of friction at end-point of the curve in order to
    keep it quasi-statically stable
    %}
    
    F_contact = [lambda.eqnonlin(2), lambda.eqnonlin(3), 0]; % contact force
    
    
    [end_normal_u, end_normal_v] = computeEndPointNormal(var_theta);
    contact_normal = [end_normal_u; end_normal_v; 0];
    
    
    cone_theta = atan2(norm(cross(F_contact,contact_normal)),dot(F_contact,contact_normal));
    
    
    CoF = tan(cone_theta);

end


