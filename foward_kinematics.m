function [pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4] = foward_kinematics(q_leg1, q_leg2, q_leg3, q_leg4)
%FOWARD_KINEMATICS Calcula a cinemática direta das quatro pernas do robô com relação ao seu centro de massa
%
%   [pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4] = foward_kinematics(q_leg1, q_leg2, q_leg3, q_leg4)
%
%   Calcula as posições finais (posição do efetuador) das quatro pernas do robô
%   a partir dos vetores de ângulos articulares fornecidos para cada perna.
%
%   Entradas:
%       q_legN - vetor coluna (3x1) com os ângulos das juntas da perna N
%
%   Saídas:
%       pos_leg_N - vetor (3x1) posição final da perna N no espaço cartesiano
%
%   Observações:
%       - A função carrega os objetos SerialLink das pernas a partir do arquivo
%         'constants.mat'.
%       - Realiza o cálculo da cinemática direta para cada perna usando os métodos
%         fkine da toolbox Robotics Toolbox.
%       - Plota os frames de cada elo das pernas e a plataforma conectando as pernas.
%       - Utiliza a função auxiliar h2e para converter coordenadas homogêneas em
%         coordenadas euclidianas.
%
%   Exemplo de uso:
%       [pos1, pos2, pos3, pos4] = foward_kinematics(q1, q2, q3, q4);
%
%   Autor: Lucca
%   Data: 2025-04-21

    load('constants.mat', 'leg1', 'leg2', 'leg3', 'leg4')   


    % Transformação homogênea plataforma-pata (posição final da perna)
    TCN_leg1 = leg1.fkine(q_leg1).T;
    TCN_leg2 = leg2.fkine(q_leg2).T;
    TCN_leg3 = leg3.fkine(q_leg3).T;
    TCN_leg4 = leg4.fkine(q_leg4).T;

    % Transformação homogênea de cada link
    T_links_leg1 = {leg1.A(1, q_leg1).T, leg1.A(2, q_leg1).T, leg1.A(3, q_leg1).T};
    T_links_leg2 = {leg2.A(1, q_leg2).T, leg2.A(2, q_leg2).T, leg2.A(3, q_leg2).T};
    T_links_leg3 = {leg3.A(1, q_leg3).T, leg3.A(2, q_leg3).T, leg3.A(3, q_leg3).T};
    T_links_leg4 = {leg4.A(1, q_leg4).T, leg4.A(2, q_leg4).T, leg4.A(3, q_leg4).T};
    
    T01 = leg1.A(1, q_leg1).T;
    T12 = leg1.A(2, q_leg1).T;
    T23 = leg1.A(3, q_leg1).T;
    
    % Plot da cinemática direta do robô
    figure;
    plot_frames_pos(leg1.base.T, T_links_leg1{1}, T_links_leg1{2}, T_links_leg1{3});
    plot_frames_pos(leg2.base.T, T_links_leg2{1}, T_links_leg2{2}, T_links_leg2{3});
    plot_frames_pos(leg3.base.T, T_links_leg3{1}, T_links_leg3{2}, T_links_leg3{3});
    plot_frames_pos(leg4.base.T, T_links_leg4{1}, T_links_leg4{2}, T_links_leg4{3});
    
    % Coordenadas dos pontos da extremidade da plataforma (4 cantos)
    pos_1 = transl(leg1.base.T)';
    pos_2 = transl(leg2.base.T)';
    pos_3 = transl(leg3.base.T)';
    pos_4 = transl(leg4.base.T)';
    x = [pos_1(1), pos_3(1), pos_4(1), pos_2(1), pos_1(1)];
    y = [pos_1(2), pos_3(2), pos_4(2), pos_2(2), pos_1(2)];
    z = [pos_1(3), pos_3(3), pos_4(3), pos_2(3), pos_1(3)];

    % Plotar a linha conectando os pontos da plataforma
    plot3(x, y, z, '-o', 'LineWidth', 2, 'Color', 'b');

    % Cálculo da posição final para cada perna
    pos_leg_1 = h2e((TCN_leg1) * [0; 0; 0; 1]);
    pos_leg_2 = h2e((TCN_leg2) * [0; 0; 0; 1]);
    pos_leg_3 = h2e((TCN_leg3) * [0; 0; 0; 1]);
    pos_leg_4 = h2e((TCN_leg4) * [0; 0; 0; 1]);
    
end