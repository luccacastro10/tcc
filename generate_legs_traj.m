function trajetoria_global_ciclos = generate_legs_traj(p_leg1, p_leg2, p_leg3, p_leg4, a, b, duracao, duracao_pausa, taxa_amostragem, ciclos)
 
pausa_inicial = [p_leg1', p_leg2', p_leg3', p_leg4', 0];
pausa_inicial = appendLastRow(pausa_inicial, 499);

[incentro1_leg1, incentro1_leg2, incentro1_leg3, incentro1_leg4] = ajuste_incentro(p_leg1, p_leg2, p_leg3, p_leg4, 1, 500);

ida1_leg1   = generate_elipse_traj(incentro1_leg1(end,:), a, b, duracao, taxa_amostragem);
volta1_leg2 = generate_elipse_traj(incentro1_leg2(end,:), a, b, duracao, taxa_amostragem, false);
volta1_leg3 = generate_elipse_traj(incentro1_leg3(end,:), a, b, duracao, taxa_amostragem, false);
volta1_leg4 = generate_elipse_traj(incentro1_leg4(end,:), a, b, duracao, taxa_amostragem, false);
n_volta = length(volta1_leg2);
terco = n_volta / 3;
volta1_leg2 = volta1_leg2(1: terco,:);
volta1_leg3 = volta1_leg3(1: terco,:);
volta1_leg4 = volta1_leg4(1: terco,:);

[incentro2_leg1, incentro2_leg2, incentro2_leg3, incentro2_leg4] = ajuste_incentro(ida1_leg1(end,:)', volta1_leg2(end,:)', volta1_leg3(end,:)', volta1_leg4(end,:)', 4, 500);

volta2_leg1 = generate_elipse_traj(incentro2_leg1(end,:), a, b, duracao, taxa_amostragem, false);
volta2_leg2 = generate_elipse_traj(incentro2_leg2(end,:), a, b, duracao, taxa_amostragem, false);
volta2_leg3 = generate_elipse_traj(incentro2_leg3(end,:), a, b, duracao, taxa_amostragem, false);
ida2_leg4   = generate_elipse_traj(incentro2_leg4(end,:), a, b, duracao, taxa_amostragem);
n_volta = length(volta2_leg1);
terco = n_volta / 3;
volta2_leg1 = volta2_leg1(1: terco,:);
volta2_leg2 = volta2_leg2(1: terco,:);
volta2_leg3 = volta2_leg3(1: terco,:);

[incentro3_leg1, incentro3_leg2, incentro3_leg3, incentro3_leg4] = ajuste_incentro(volta2_leg1(end,:)', volta2_leg2(end,:)', volta2_leg3(end,:)', ida2_leg4(end,:)', 3, 500);

volta3_leg1 = generate_elipse_traj(incentro3_leg1(end,:), a, b, duracao, taxa_amostragem, false);
volta3_leg2 = generate_elipse_traj(incentro3_leg2(end,:), a, b, duracao, taxa_amostragem, false);
ida3_leg3   = generate_elipse_traj(incentro3_leg3(end,:), a, b, duracao, taxa_amostragem);
volta3_leg4 = generate_elipse_traj(incentro3_leg4(end,:), a, b, duracao, taxa_amostragem, false);
n_volta = length(volta3_leg1);
terco = n_volta / 3;
volta3_leg1 = volta3_leg1(1: terco,:);
volta3_leg2 = volta3_leg2(1: terco,:);
volta3_leg4 = volta3_leg4(1: terco,:);

[incentro4_leg1, incentro4_leg2, incentro4_leg3, incentro4_leg4] = ajuste_incentro(volta3_leg1(end,:)', volta3_leg2(end,:)', ida3_leg3(end,:)', volta3_leg4(end,:)', 2, 500);

volta4_leg1 = generate_elipse_traj(incentro4_leg1(end,:), a, b, duracao, taxa_amostragem, false);
ida4_leg2   = generate_elipse_traj(incentro4_leg2(end,:), a, b, duracao, taxa_amostragem);
volta4_leg3 = generate_elipse_traj(incentro4_leg3(end,:), a, b, duracao, taxa_amostragem, false);
volta4_leg4 = generate_elipse_traj(incentro4_leg4(end,:), a, b, duracao, taxa_amostragem, false);
n_volta = length(volta4_leg1);
terco = n_volta / 3;
volta4_leg1 = volta4_leg1(1: terco,:);
volta4_leg3 = volta4_leg3(1: terco,:);
volta4_leg4 = volta4_leg4(1: terco,:);


traj_1 = [incentro1_leg1, incentro1_leg2, incentro1_leg3, incentro1_leg4, 0*ones(size(ida1_leg1(:,1)))];
traj_2 = [ida1_leg1,  volta1_leg2,  volta1_leg3, volta1_leg4, 1*ones(size(ida1_leg1(:,1)))];
traj_2 = appendLastRow(traj_2, 100);
traj_3 = [incentro2_leg1, incentro2_leg2, incentro2_leg3, incentro2_leg4, 0*ones(size(ida1_leg1(:,1)))];
traj_4 = [volta2_leg1,  volta2_leg2,  volta2_leg3, ida2_leg4, 4*ones(size(ida1_leg1(:,1)))];
traj_4 = appendLastRow(traj_4, 100);
traj_5 = [incentro3_leg1, incentro3_leg2, incentro3_leg3, incentro3_leg4, 0*ones(size(ida1_leg1(:,1)))];
traj_6 = [volta3_leg1,  volta3_leg2,  ida3_leg3, volta3_leg4, 3*ones(size(ida1_leg1(:,1)))];
traj_6 = appendLastRow(traj_6, 100);
traj_7 = [incentro4_leg1, incentro4_leg2, incentro4_leg3, incentro4_leg4, 0*ones(size(ida1_leg1(:,1)))];
traj_8 = [volta4_leg1,  ida4_leg2,  volta4_leg3, volta4_leg4, 2*ones(size(ida1_leg1(:,1)))];

p0 = traj_8(end, :);
% Ponto final: posições atuais das pernas empilhadas
pf = [p_leg1', p_leg2', p_leg3', p_leg4', 0]';

% Interpolação linear
traj_9 = zeros(500, length(p0));
for i = 1:length(p0)-1
    traj_9(:,i) = linspace(p0(i), pf(i), 500)';
end

% perna 1 -> perna 4 -> perna 3 -> perna 2
trajetoria_global = [pausa_inicial; traj_1; traj_2; traj_3; traj_4; traj_5; traj_6; traj_7; traj_8; traj_9];



% tempo = linspace(0, duracao_pausa/2+ciclos*4*(duracao/2), duracao_pausa/2 * taxa_amostragem+ciclos*round(4*(duracao/2) * taxa_amostragem))'; % para dar 4 passos
tempo = linspace(0, ciclos*10.6*(duracao/2), ciclos*round(10.6*(duracao/2) * taxa_amostragem))'; % para dar 4 passos

trajetoria_global_ciclos = []; % Ou trajetoria_global = trajetoria_original;
trajetoria_global_ciclos = repmat(trajetoria_global, ciclos, 1);
% trajetoria_global_ciclos = [traj_pausa; trajetoria_global_ciclos];
trajetoria_global_ciclos = [tempo, trajetoria_global_ciclos];

% check_com_stability_visual(trajetoria_global_ciclos);

% trajetoria_global_ciclos = trajetoria_global_ciclos(:, 1:13); % remove coluna identificadora de pernas levantando

end

function check_com_stability_visual(traj)
    num_rows = size(traj, 1);

    figure;
    axis equal;
    grid on;
    hold on;
    title('Verificação da Estabilidade do CoM');
    xlabel('X [m]');
    ylabel('Y [m]');

    for i = 1:num_rows
        row = traj(i, :);
        t = row(1);
        leg_up = row(14);

        % Posições das pernas (4x3)
        legs = reshape(row(2:13), 3, 4)';  % [x y z] para cada perna

        % Caso especial: todas as patas no chão (leg_up = 0)
        if leg_up == 0
            support_legs = 1:4; % Todas as patas são de suporte
            support_xy = legs(:, 1:2); % Todas as posições XY
            
            % Plotar todas as patas como apoio
            plot(support_xy(:,1), support_xy(:,2), 'ko', 'MarkerSize', 8, 'LineWidth', 1.5);
            
            % Não há pata levantada para plotar
        else
            % Caso normal: uma pata levantada
            support_legs = setdiff(1:4, leg_up);
            support_xy = legs(support_legs, 1:2);
            
            % Plotar patas de apoio
            plot(support_xy(:,1), support_xy(:,2), 'ko', 'MarkerSize', 8, 'LineWidth', 1.5);
            
            % Plotar pata levantada
            leg_up_pos = legs(leg_up, 1:2);
            plot(leg_up_pos(1), leg_up_pos(2), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
        end

        % Verifica se o CoM (0,0) está dentro do polígono de suporte
        is_inside = inpolygon(0, 0, support_xy(:,1), support_xy(:,2));

        % Cor da linha: verde se estável, vermelho se instável
        poly_color = 'g';
        if ~is_inside
            poly_color = 'r';
        end

        % Plotar polígono de sustentação (apenas se tiver 3+ pontos)
        if size(support_xy,1) >= 3
            fill([support_xy(:,1); support_xy(1,1)], ...
                 [support_xy(:,2); support_xy(1,2)], ...
                 poly_color, 'FaceAlpha', 0.3, 'EdgeColor', poly_color);
        end

        % Plotar CoM
        plot(0, 0, 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'k');

        % Atualizar título com tempo
        title(sprintf('t = %.2f s — %s (Pés no chão: %d)', t, ...
              ternary(is_inside, 'Estável', 'Instável'), ...
              length(support_legs)));

        pause(0.0001);  % controle de tempo para visualização
        cla;  % limpa o gráfico para o próximo frame
    end
end

function out = ternary(cond, val_true, val_false)
    if cond
        out = val_true;
    else
        out = val_false;
    end
end

function [traj_p1, traj_p2, traj_p3, traj_p4] = ajuste_incentro(perna1, perna2, perna3, perna4, perna_levantar, n_pontos)
    
    perna1 = perna1';
    perna2 = perna2';
    perna3 = perna3';
    perna4 = perna4';

    % Organiza todas as pernas (já transpostas se necessário)
    pernas = [perna1; perna2; perna3; perna4];
    
    % Identifica as pernas de apoio
    idx_apoio = setdiff(1:4, perna_levantar);
    pernas_apoio = pernas(idx_apoio, :);
    
    % Calcula o vetor para o incentro
    vetor_incentro = calcula_vetor_incentro(pernas_apoio(1,:), pernas_apoio(2,:), pernas_apoio(3,:));
    deslocamento = vetor_incentro(1:2); % Apenas XY
    
    % Prepara as trajetórias de saída
    traj_p1 = zeros(n_pontos, 3);
    traj_p2 = zeros(n_pontos, 3);
    traj_p3 = zeros(n_pontos, 3);
    traj_p4 = zeros(n_pontos, 3);
    
    % Gera os pontos das trajetórias
    for i = 1:n_pontos
        fator = i/n_pontos;
        desloc = [fator*deslocamento, 0];
        
        % Aplica o mesmo deslocamento a TODAS as pernas (incluindo a que será levantada)
        traj_p1(i,:) = perna1 - [0.25*desloc(1), desloc(2), 0];
        traj_p2(i,:) = perna2 - [0.25*desloc(1), desloc(2), 0];
        traj_p3(i,:) = perna3 - [0.25*desloc(1), desloc(2), 0];
        traj_p4(i,:) = perna4 - [0.25*desloc(1), desloc(2), 0];
    end
    
    % Agora todas as pernas (incluindo a que será levantada) são movidas igualmente
    % Isso mantém a coerência do movimento do corpo como um todo
end

function vetor_incentro = calcula_vetor_incentro(p1, p2, p3)
    % Pega apenas coordenadas XY
    pts = [p1(1:2); p2(1:2); p3(1:2)];
    
    % Calcula lados do triângulo
    a = norm(pts(2,:) - pts(3,:));
    b = norm(pts(1,:) - pts(3,:));
    c = norm(pts(1,:) - pts(2,:));
    
    % Calcula incentro
    incentro = (a*pts(1,:) + b*pts(2,:) + c*pts(3,:)) / (a + b + c);
    
    % Vetor do CoM (0,0) ao incentro
    vetor_incentro = [incentro(1), incentro(2), 0];
end

function A = appendLastRow(A, n)
    % Obtém a última linha da matriz
    last_row = A(end, :);  
    % Repete a última linha 'n' vezes
    new_rows = repmat(last_row, n, 1);  
    % Concatena as novas linhas à matriz original
    A = [A; new_rows];  
end
