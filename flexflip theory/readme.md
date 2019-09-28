# Modeling for ***flexflip***


### Generate a minimum bending energy curve

First, define the objective function. We intend to minimize the square of the curvature along the length of the curve. This is proportional to the bending energy of the curve.
```Matlab
function fvalue = objectiveFunction(var_theta, var_s)

    dthetads = gradient(var_theta)./gradient(var_s); %curvature

    fvalue = trapz(var_s, dthetads.^2);
end
```

Specify start and end points constraint. Note, the start point is clamped. End point tangent is unspecified.
```Matlab
function [c_ineq, c_eq] = constraintFunctions(var_theta, var_s, curve_props)
  
    % start point tangent constraint
    c_eq_1 = var_theta(1)- curve_props.startPointSlope;


    % end point position constraint
    c_eq_3 = trapz(var_s, cos(var_theta))- curve_props.endPoint(1);
    c_eq_4 = trapz(var_s, sin(var_theta))- curve_props.endPoint(2);

    c_eq = [c_eq_1; c_eq_3; c_eq_4];

    c_ineq = []; %no inequality constraints.
end
```

Provide an initial guess.
```Matlab
var_theta_init = 1*ones(1, length(var_s)) +...
                 1*sin(2*pi*var_s/var_s(end)) +...
                 1*cos(2*pi*var_s/var_s(end)) +...
                 1*sin(4*pi*var_s/var_s(end)) +...
                 1*cos(4*pi*var_s/var_s(end)); %initial guess for theta(s)
```

Solve optimization problem
```Matlab
options = optimoptions('fmincon',...
                       'Algorithm',...
                       'interior-point');

options.MaxFunctionEvaluations = 1e5;
options.OptimalityTolerance= 1e-2; 
options.StepTolerance = 1e-3;

[var_theta,fval,exitflag,output,lambda,grad,hessian] = ...
                fmincon(@(var_theta)objectiveFunction(var_theta, var_s),...
                var_theta_init,...
                [],[],[],[],[],[],...
                @(var_theta)constraintFunctions(var_theta, var_s, curve_props),...
                options);
```
 
Finally, express the curve in Cartesian coordinates.
```Matlab
    % return x,y coordinates of the bending curve
    xc = cumtrapz(var_s, cos(var_theta));
    yc = cumtrapz(var_s, sin(var_theta));   
```

Show an example curve here.


### (Minimum) coefficient of friction to keep the curve steady
The Lagrange multipliers corresponding to the end-point constraints on the curve can be used to determine the minimum coefficient of friction required at the end-point to keep curve in quasi-static equilibrium.  

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

