function plotResults(X, Y, U_flex_matrix, CoF_matrix, Tip_normal_matrix_U, Tip_normal_matrix_V, curves_axis)
    %{
    Plot flexure energy, coefficient of friction, and end-point normal
    vectors overlayed on to the curves plot.
    %}

    % convert zeros to NaN so they don't showup in the plots
    U_flex_matrix(U_flex_matrix==0) = NaN; 
    CoF_matrix(CoF_matrix==0) = NaN;
    Tip_normal_matrix_U(Tip_normal_matrix_U==0) = NaN;
    Tip_normal_matrix_V(Tip_normal_matrix_V==0) = NaN;


    % plot flexure energy, cof, and contact normals.
    energy_figure = figure(2);
    energy_axis = copyobj(curves_axis, energy_figure);
    hold on
    surf(X,Y,U_flex_matrix);
    colorbar
    title('Bending Energy')
    grid on


    cof_figure = figure(3);
    cof_axis = copyobj(curves_axis, cof_figure);
    hold on
    surf(X,Y,CoF_matrix);
    colorbar
    title('Coefficient of Friction')
    grid on


    normals_figure = figure(4);
    normals_axis = copyobj(curves_axis, normals_figure);
    hold on
    quiver(X,Y,Tip_normal_matrix_U, Tip_normal_matrix_V);
    title('Contact Normals')
    axis equal
    grid on
end

