function [xc,yc,var_theta,lambda] = bendingCurveFun(x_end, y_end, bool_plot_curve)

    %{
        This functions outputs a minimum bending energy curve given the
        boundary conditions at two end-points. Here one end is clamped at
        origin (zero slope) while the coordinates of other end form the 
        the argument of this function i.e. x_end and y_end. To specify 
        boundary slopes modify the function 'constraintFunctions' below. 

        We use fmincon to solve the resulting constrained optimization 
        problem. The curve is assumed to have a unit length. The problem is 
        solved in arclength-theta coordinates. The optimization variable is
        the slope of the curve at each discretized value of the arclength. 
        The algorithm chooses the slope such that the total bending energy 
        of the curve can be minimized. The function outputs the minimal 
        energy curve in cartesian form. Set plot to True to plot the 
        resulting curve.

        eg. [xc,yc] = bendingCurveFun(0.5, 0.5, true)

    %}

    intervals = 70; %for arclength discretization.

    arc_length = 1; %Fixed curve length constraint

    var_s = linspace(0, arc_length, intervals); %arclength variable

    var_theta_init = 1*ones(1, length(var_s)) +...
                     1*sin(2*pi*var_s/var_s(end)) +...
                     1*cos(2*pi*var_s/var_s(end)) +...
                     1*sin(4*pi*var_s/var_s(end)) +...
                     1*cos(4*pi*var_s/var_s(end)); %Initial guess for theta(s)


    options = optimoptions('fmincon',...
                           'Algorithm',...
                           'interior-point');

    options.MaxFunctionEvaluations = 100000000;
    options.OptimalityTolerance= 1e-2; % important parameter
    options.StepTolerance = 1e-3; % important parameter

    [var_theta,fval,exitflag,output,lambda,grad,hessian] = ...
                    fmincon(@objectiveFunction,...
                    var_theta_init,...
                    [],[],[],[],[],[],...
                    @constraintFunctions,...
                    options);

                
    [xc,yc] = arcLengthToCartesian(var_theta, var_s);


    if bool_plot_curve
        
        plot(xc,yc)
        
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function fvalue = objectiveFunction(var_theta)

        dthetads = computeDifferentials(var_theta, var_s);
        fvalue = trapz(var_s, dthetads.^2);

    end


    function [c, c_eq] = constraintFunctions(var_theta)
        %This function defines equality and inequality constraints on the 
        %optimization problem. 

        % end point tangent constraint
        c_eq_1 = var_theta(1);
        % c_eq_2 = var_theta(end);


        % end point position constraint (arc length coordinates)
        c_eq_3 = trapz(var_s, cos(var_theta))-x_end;
        c_eq_4 = trapz(var_s, sin(var_theta))-y_end;

        c_eq = [c_eq_1; c_eq_3; c_eq_4];

        c = []; %no inequality constraints.

    end


    function dthetads = computeDifferentials(var_theta, var_s)

        dthetads = gradient(var_theta)./gradient(var_s);

    end


    function [xc, yc] = arcLengthToCartesian(var_theta, var_s)
        
        xc = cumtrapz(var_s, cos(var_theta));
        yc = cumtrapz(var_s, sin(var_theta));
        
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
