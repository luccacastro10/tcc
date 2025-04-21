% Carregar as constantes
load('constants.mat', 'leg1', 'leg2', 'leg3', 'leg4');

% Configuração das juntas da perna
q0_leg1 = [+pi/9; +pi/6; -pi/3];
q0_leg2 = [+pi/9; +pi/6; -pi/3];
q0_leg3 = [-pi/9; -pi/6; +pi/3];
q0_leg4 = [-pi/9; -pi/6; +pi/3];

wn = pi/2;          % Frequência angular (rad/s)
r = 400;            % Raio da circunferência (m)
x0 = [0; 0; 0];     % Centro da circunferência no plano X-Z

% Tempo para uma volta completa (2*pi/wn)
t = linspace(0, 2*pi/wn, 500);  % 100 pontos para uma volta completa

% Inicializar arrays para armazenar as trajetórias
xd_traj = zeros(3, length(t)); % Trajetória xd
xd_leg1_traj = zeros(3, length(t)); % Trajetória perna 1
xd_leg2_traj = zeros(3, length(t)); % Trajetória perna 2
xd_leg3_traj = zeros(3, length(t)); % Trajetória perna 3
xd_leg4_traj = zeros(3, length(t)); % Trajetória perna 4

% Definir a altura do passo usando uma curva de Bézier quadrática
quantidade_passos = 10;
altura_maxima = 20; % Altura máxima do passo
t_bezier = linspace(-sqrt(altura_maxima), +sqrt(altura_maxima), length(t)/quantidade_passos); % Parâmetro t para a curva de Bézier
altura_passo = altura_maxima - t_bezier.^2; % Curva de Bézier quadrática

perna_da_vez = 1;
i_passo = 1;
% Lógica para alternar o movimento das pernas
for i = 1:length(t)
    % Trajetória no plano X-Y
    xd = [x0(1) + r*cos(wn*t(i));
          x0(2) + r*sin(wn*t(i));
          x0(3)];
      
    xd_dot = [0 - r*wn*sin(wn*t(i));
              0 + r*wn*cos(wn*t(i));
              0];
      
    % Calcular o ângulo em relação ao eixo X usando atan2
    theta_rad = atan2(xd_dot(2), xd_dot(1)); % Retorna valores entre -pi e pi

    % Ajustar o ângulo para o intervalo [0, 2*pi]
    if theta_rad < 0
        theta_rad = theta_rad + 2*pi;
    end

    % Converter o ângulo para graus (opcional)
    theta_deg = rad2deg(theta_rad);
    
    % Armazenar a trajetória xd
    xd_traj(:, i) = xd;
    
    % Extrair as posições (x, y, z) das transformações homogêneas
    p_leg1 = h2e(transl(xd) * trotz(theta_deg) * e2h(transl(leg1.fkine(q0_leg1).T)));
    p_leg2 = h2e(transl(xd) * trotz(theta_deg) * e2h(transl(leg2.fkine(q0_leg2).T)));
    p_leg3 = h2e(transl(xd) * trotz(theta_deg) * e2h(transl(leg3.fkine(q0_leg3).T)));
    p_leg4 = h2e(transl(xd) * trotz(theta_deg) * e2h(transl(leg4.fkine(q0_leg4).T)));
    
    i_passo = i_passo+1;
    
    if i_passo == length(altura_passo)
        i_passo = 1;
        perna_da_vez = perna_da_vez + 1;
        if perna_da_vez == 5
            perna_da_vez = 1;
        end
    end
    
    switch perna_da_vez
        case 1
            p_leg1(3) = p_leg1(3) + altura_passo(i_passo);
        case 2
            p_leg2(3) = p_leg2(3) + altura_passo(i_passo);
        case 3
            p_leg3(3) = p_leg3(3) + altura_passo(i_passo);
        case 4
            p_leg4(3) = p_leg4(3) + altura_passo(i_passo);
    end
    
    % Armazenar as trajetórias das pernas
    xd_leg1_traj(:, i) = p_leg1;
    xd_leg2_traj(:, i) = p_leg2;
    xd_leg3_traj(:, i) = p_leg3;
    xd_leg4_traj(:, i) = p_leg4;
end

% Plotar as trajetórias no espaço 3D
figure;
hold on;
plot3(xd_traj(1, :), xd_traj(2, :), xd_traj(3, :), 'k', 'LineWidth', 2); % Trajetória xd
plot3(xd_leg1_traj(1, :), xd_leg1_traj(2, :), xd_leg1_traj(3, :), 'r'); % Trajetória perna 1
plot3(xd_leg2_traj(1, :), xd_leg2_traj(2, :), xd_leg2_traj(3, :), 'g'); % Trajetória perna 2
plot3(xd_leg3_traj(1, :), xd_leg3_traj(2, :), xd_leg3_traj(3, :), 'b'); % Trajetória perna 3
plot3(xd_leg4_traj(1, :), xd_leg4_traj(2, :), xd_leg4_traj(3, :), 'm'); % Trajetória perna 4

xlabel('X');
ylabel('Y');
zlabel('Z');
title('Trajetórias no espaço 3D - Passo de Cachorro');
legend('xd', 'Perna 1', 'Perna 2', 'Perna 3', 'Perna 4');
grid on;
hold off;