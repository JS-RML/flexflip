% Create a (minimum) bending energy curve

clear all 
close all


addpath('./modules')


% bending curve parameters:
curve_props.length = 1; % s
curve_props.endPoint = [0.6, 0.3]; %[x_end, y_end]
curve_props.startPointSlope = 0; %dy/dx @ s=0; or \theta @ s=0

% create arc length variable
intervals = 70; %for arclength discretization.
var_s = linspace(0, curve_props.length, intervals); % arclength variable

[xc,yc,var_theta,lambda] = generateBendingCurve(var_s, curve_props);