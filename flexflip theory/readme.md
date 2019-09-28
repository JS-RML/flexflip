# Modeling for ***flexflip***




### Generate minimum bending energy curves

Find a curve that minimizes total curvature along the length of the curve.

In the file `generateBendingCurve.m`:

* Specify initial guess as a linear combination of basis functions
```Matlab
var_theta_init = 1*ones(1, length(var_s)) +...
                 1*sin(2*pi*var_s/var_s(end)) +...
                 1*cos(2*pi*var_s/var_s(end)) +...
                 1*sin(4*pi*var_s/var_s(end)) +...
                 1*cos(4*pi*var_s/var_s(end));
```


### (Minimum) coefficient of friction to keep the curve steady
The Lagrange multipliers corresponding to the end-point constraints on the curve can be used to determine the minimum coefficient of friction required at the end-point to keep object in quasi-static equilibrium.  
```Matlab

function CoF = computeCoF(lambda, var_theta)
    %{
    Compute coefficient of friction at end-point of the curve in order to
    keep it quasi-statically stable
    INPUT:
    lamda: Lagrange multipliers corresponding to end-point constraint
    var_theta: variable theta as a function of arc length
    OUTPUT:
    CoF: coefficient of friction
    %}
    
    F_contact = [lambda.eqnonlin(2), lambda.eqnonlin(3), 0]; % contact/constraint force
    
    
    [end_normal_u, end_normal_v] = computeEndPointNormal(var_theta);
    contact_normal = [end_normal_u; end_normal_v; 0];
    
    
    cone_theta = atan2(norm(cross(F_contact,contact_normal)),dot(F_contact,contact_normal));
    
    
    CoF = tan(cone_theta);

end
```
