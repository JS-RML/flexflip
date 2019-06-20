
%{
Obtain bending energy curves over the space of the right end-point domain.
Here we assume that the left end is clamped. 
For each curve, we compute:
1. Total bending energy (also called the flexural energy)
2. Concave-up and Concave-down bending energies based on the inflexion point
3. Minimum coefficient of friction (CoF) required at the right contact to 
maintain static equilibrium with the object. This is computed based on the
Lagrange multipliers that help enforce the end-point constraint and the 
curve normal at the right end point. The latter determines the 'contact
normal'.

March 24, 2019
%}

close all
clear all

addpath('./readonly')

global var_s x_end y_end

%%%%%%%These values should be same as in the bending energy function%%%%%%%
arc_length = 1; %Conisder unit length curves
intervals = 70; %for arclength discretization.
var_s = linspace(0, arc_length, intervals);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

size = 20; %Control discretization of end-points

% Set range of end-points
x_bound = [0.3*arc_length, 0.95*arc_length];
y_bound = [0.0,0.5*arc_length];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (OPTIONAL): Discretization step size between end-point coordinates. 
%Useful when calculating gradient vector fields of flexural energy etc. 
%The vector fields can be visualized with a quiver plot.
hx = (x_bound(2)-x_bound(1))/(size-1);
hy = (y_bound(2)-y_bound(1))/(size-1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[X, Y] = sampleEnergyDomain(x_bound, y_bound, size);

%Containers to store concave-up, concave-down and total flexure energy
U_flex_matrix = zeros(size);
U_flex_CU_matrix = zeros(size);
U_flex_CD_matrix = zeros(size);

%Container to store coefficient of friction
CoF_matrix = zeros(size);

%Container to store object tip normal
Tip_normal_matrix_U = zeros(size);
Tip_normal_matrix_V = zeros(size);

%Only a triangular region for end-points is of interest here
ones_lt_flipped = flip(tril(ones(size),2));
ut_idxes = find(ones_lt_flipped>0);
 
figure(1); % For curves and flexural energy

for xidx = ut_idxes.'

        x_end = X(xidx);
        y_end = Y(xidx);
        

        [xc,yc,var_theta,lambda] = bendingCurveFun(x_end, y_end, false);

        plot(xc,yc)
        drawnow
        hold on
        
        % Compute total flexure energy
        U_flex_matrix(xidx) = computeFlexuralEnergy(var_theta, var_s);
        
        % Compute concave up and concave down flexure energies
        [U_flex_CU_matrix(xidx), U_flex_CD_matrix(xidx)] = compute_CUP_CAP_energies(var_theta, var_s);

        % Compute minimum coefficient of friction
        CoF_matrix(xidx) = computeCoF(lambda.eqnonlin(2),lambda.eqnonlin(3), var_theta(end));

        % Compute the curve normal at the end point. This can be visualized
        % using quiver plot.
        [Tip_normal_matrix_U(xidx), Tip_normal_matrix_V(xidx)] = compute_end_normal(var_theta);



end

%Convert zeros to NaN
U_flex_matrix(U_flex_matrix==0) = NaN; 
U_flex_CU_matrix(U_flex_CU_matrix==0) = NaN; 
U_flex_CD_matrix(U_flex_CD_matrix==0) = NaN; 
CoF_matrix(CoF_matrix==0) = NaN;
Tip_normal_matrix_U(Tip_normal_matrix_U==0) = NaN;
Tip_normal_matrix_V(Tip_normal_matrix_V==0) = NaN;


%PLOT: Either plot U_flex_matrix or CoF Friction on top of the curves plot.
%You can also save the computed quantities and use for secondary tasks.
surf(X,Y,U_flex_matrix);
% surf(X,Y,CoF_matrix);
% hold on

% plot_boundary(arc_length) %unit length boundary
axis equal


% % (OPTIONAL): Compute the gradient vector field corresponding to total
% % flexural energy etc.
% [gx,gy] = computeNumericalGradient(U_flex_matrix, hx, hy);
% figure(2)
% quiver(X,Y,-gx,-gy)
% axis equal


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plot_boundary(arc_length)

    t = linspace(0,2*pi/3,100);
    xt = arc_length*cos(t);
    yt = arc_length*sin(t);
    plot(xt, yt)
    hold on

end

function [normal_u, normal_v] = compute_end_normal(var_theta)

    if var_theta(end)<=0
        normal_u = -sin(var_theta(end));
        normal_v = cos(var_theta(end));
    else
        normal_u = sin(var_theta(end));
        normal_v = -cos(var_theta(end));
    end

end


function dthetads = computeDifferentials(var_theta, var_s)

    dthetads = gradient(var_theta)./gradient(var_s);
    
end



function U_flex = computeFlexuralEnergy(var_theta, var_s)
    %this function computes flexural energy of a curve based on the 
    %curvature, without factoring the rigidity constraint.
    
    dthetads = computeDifferentials(var_theta, var_s);
    
    U_flex = trapz(var_s, dthetads.^2);
    
end

function [gx,gy] = computeNumericalGradient(F, hx, hy)

    %F is the functional matrix, and h is the uniform spacing
    [gx,gy] = gradient(F, hx, hy);
        
end

function CoF = computeCoF(Fx,Fy, ang)
    
    %First compute the magnitude and direction of contact force
    
    F_contact = [Fx, Fy, 0];
    
    %To ensure contact normal always points outward
    if ang<=0
        contact_normal = [-sin(ang), cos(ang), 0];
    else
        contact_normal = [sin(ang), -cos(ang), 0];
    end

    cone_theta = atan2(norm(cross(F_contact,contact_normal)),dot(F_contact,contact_normal));
    
    CoF = tan(cone_theta);



end


function [U_flex_CU, U_flex_CD] = compute_CUP_CAP_energies(var_theta, var_s)

    smoothed_var_theta = smooth(var_theta, 'rlowess');
    max_logical_vec = islocalmax(smoothed_var_theta).';
    max_indices = find(max_logical_vec == 1, 1, 'first');
    
    if numel(max_indices) == 1
        
        U_flex_CU = computeFlexuralEnergy(...
                            var_theta(1:max_indices),...
                            var_s(1:max_indices));
                        
        U_flex_CD = computeFlexuralEnergy(...
                            var_theta(1+max_indices:end),...
                            var_s(1+max_indices:end));
        
        
    else
        disp('Inflection Issues')
        U_flex_CU = computeFlexuralEnergy(var_theta, var_s);
        U_flex_CD = 0;
    end
    
end



function [X, Y] = sampleEnergyDomain(x_bound, y_bound, size)

    x = linspace(x_bound(1), x_bound(2), size);
    y = linspace(y_bound(1), y_bound(2), size);
    
    [X,Y] = meshgrid(x,y);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%