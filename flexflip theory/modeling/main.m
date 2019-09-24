%{
This script plots minimum bending energy curves whose left end is clamped,
while the right end is varied within a triangular region. The corresponding 
flexure energy distribution, and the minimum coefficient of friction
required (at the right end of the curve) to maintain quasi-statibility is
also computed.

Sep. 24, 2019
%}

close all
clear all

addpath('./modules')

% bending curve parameters:
curve_props.length = 1; % s
curve_props.endPoint = []; %[x_end, y_end]
curve_props.startPointSlope = 0; %dy/dx @ s=0; or \theta @ s=0

% create arc length variable
intervals = 70; %for arclength discretization.
var_s = linspace(0, curve_props.length, intervals); % arclength variable


% create a domain for end-points
steps = 10;  % increase the step size to have more dense plot
x_bound = [0.3*curve_props.length, 0.95*curve_props.length];
y_bound = [0.0,0.5*curve_props.length];

x = linspace(x_bound(1), x_bound(2), steps);
y = linspace(y_bound(1), y_bound(2), steps);

[X,Y] = meshgrid(x,y);


% containers to store bending energies, CoFs, tip normal vectors
U_flex_matrix = zeros(steps);

CoF_matrix = zeros(steps);

Tip_normal_matrix_U = zeros(steps);
Tip_normal_matrix_V = zeros(steps);


% ... but only a triangular region for end-points is of interest to us
ones_lt_flipped = flip(tril(ones(steps),2));
ut_idxes = find(ones_lt_flipped>0);
 
curves_figure = figure(1);
curves_axis = gca;


for xidx = ut_idxes.'

        curve_props.endPoint(1) = X(xidx);
        curve_props.endPoint(2) = Y(xidx);
        
        [xc,yc,var_theta,lambda] = generateBendingCurve(var_s, curve_props);
       
        % compute total flexure energy
        U_flex_matrix(xidx) = computeFlexureEnergy(var_s, var_theta);
        
        % compute minimum coefficient of friction
        CoF_matrix(xidx) = computeCoF(lambda, var_theta);

        % compute the curve normal at the end point
        [Tip_normal_matrix_U(xidx), Tip_normal_matrix_V(xidx)] = computeEndPointNormal(var_theta);

end


%plot results
plotResults(X, Y, U_flex_matrix, CoF_matrix, Tip_normal_matrix_U, Tip_normal_matrix_V, curves_axis)


