function plot_frames_pos(TIC, TC_0N, T0_1, T1_2, T2_3, hold_on_flag)
    points = [transl(TIC)';
              transl(TIC*TC_0N)';
              transl(TIC*TC_0N*T0_1)';
              transl(TIC*TC_0N*T0_1*T1_2)';
              transl(TIC*TC_0N*T0_1*T1_2*T2_3)'];

    if size(points, 2) ~= 3
        error('Os pontos devem ter exatamente três colunas (X, Y, Z).');
    end

    if hold_on_flag
        hold on;
    else
        figure;
    end

    plot3(points(:, 1), points(:, 2), points(:, 3), 'o-', 'MarkerSize', 6, 'LineWidth', 1.5);
    grid on;

    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Pontos no Espaço 3D');
    axis equal;

    if ~hold_on_flag
        hold off;
    end
end