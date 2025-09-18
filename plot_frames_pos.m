function plot_frames_pos(TC_0N, T0_1, T1_2, T2_3)
    
    % Posições dos pontos (origens dos frames)
    points = [transl(eye(4))';
              transl(TC_0N)';
              transl(TC_0N*T0_1)';
              transl(TC_0N*T0_1*T1_2)';
              transl(TC_0N*T0_1*T1_2*T2_3)'];
    
    % Matrizes de transformação correspondentes a cada ponto
    Ts = {TC_0N, TC_0N*T0_1, TC_0N*T0_1*T1_2, TC_0N*T0_1*T1_2*T2_3};

    if size(points, 2) ~= 3
        error('Os pontos devem ter exatamente três colunas (X, Y, Z).');
    end

    hold on;

    % Plot dos pontos
    plot3(points(:, 1), points(:, 2), points(:, 3), 'o-', 'MarkerSize', 6, 'LineWidth', 1.5);
    grid on;

    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Modelo Cinemático do Robô Quadrúpede');
    axis equal;

    % Desenha os sistemas de coordenadas associados a cada ponto
    for i = 1:length(Ts)
        plot_individual_frame(Ts{i});
    end
end

