function plot_individual_frame(T)

    load('constants.mat', 'C')
    axis_length = 0.1*C; % Tamanho das setas para os eixos

    % Extraindo as direções dos eixos X, Y e Z a partir da matriz de transformação
    origin = transl(T);              % Origem do frame
    x_axis = T(1:3, 1) * axis_length; % Eixo X
    y_axis = T(1:3, 2) * axis_length; % Eixo Y
    z_axis = T(1:3, 3) * axis_length; % Eixo Z

    % Plotando os eixos X, Y e Z
    quiver3(origin(1), origin(2), origin(3), x_axis(1), x_axis(2), x_axis(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Eixo X em vermelho
    quiver3(origin(1), origin(2), origin(3), y_axis(1), y_axis(2), y_axis(3), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Eixo Y em verde
    quiver3(origin(1), origin(2), origin(3), z_axis(1), z_axis(2), z_axis(3), 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Eixo Z em azul

    % Adicionando labels nas pontas das setas
    text(origin(1) + x_axis(1), origin(2) + x_axis(2), origin(3) + x_axis(3), 'X', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
    text(origin(1) + y_axis(1), origin(2) + y_axis(2), origin(3) + y_axis(3), 'Y', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
    text(origin(1) + z_axis(1), origin(2) + z_axis(2), origin(3) + z_axis(3), 'Z', 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');
end