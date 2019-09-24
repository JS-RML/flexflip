function [xc,yc,var_theta,lambda] = generateBendingCurve(var_s, curve_props)

    %{
    Generate minimum bending energy curve subject to start/end point constraints
    %}

    var_theta_init = 1*ones(1, length(var_s)) +...
                     1*sin(2*pi*var_s/var_s(end)) +...
                     1*cos(2*pi*var_s/var_s(end)) +...
                     1*sin(4*pi*var_s/var_s(end)) +...
                     1*cos(4*pi*var_s/var_s(end)); %initial guess for theta(s)


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

                

    % return x,y coordinates of the bending curve
    xc = cumtrapz(var_s, cos(var_theta));
    yc = cumtrapz(var_s, sin(var_theta));
    
    plot(xc,yc)
    drawnow
    hold on

end

function fvalue = objectiveFunction(var_theta, var_s)

    dthetads = gradient(var_theta)./gradient(var_s);

    fvalue = trapz(var_s, dthetads.^2);

end


function [c_ineq, c_eq] = constraintFunctions(var_theta, var_s, curve_props)
  
    % start point tangent constraint
    c_eq_1 = var_theta(1)- curve_props.startPointSlope;


    % end point position constraint
    c_eq_3 = trapz(var_s, cos(var_theta))- curve_props.endPoint(1);
    c_eq_4 = trapz(var_s, sin(var_theta))- curve_props.endPoint(2);

    c_eq = [c_eq_1; c_eq_3; c_eq_4];

    c_ineq = []; %no inequality constraints.

end






