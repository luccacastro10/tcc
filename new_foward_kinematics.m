function [pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4] = new_foward_kinematics(pos, rpy, q_leg1, q_leg2, q_leg3, q_leg4)
    
    load('constants.mat','C', 'L', 'left_legs', 'right_legs')   
    
    %left_legs.teach
    %right_legs.teach
    
    %left_legs.plot(q_leg1);
    %J = left_legs.jacob0(q_leg1);

    % Transformação homogênea inercial-centro
    TIC = transl(pos) * rpy2tr(rpy);

    % Transformações homogêneas do centro para os links iniciais de cada perna
    TC_01 = [0, 0, 1, +C/2; 0, 1, 0, +L/2; -1, 0, 0, 0; 0, 0, 0, 1]; % centro -> perna dianteira/esquerda
    TC_02 = [0, 0, 1, -C/2; 0, 1, 0, +L/2; -1, 0, 0, 0; 0, 0, 0, 1]; % centro -> perna traseira/esquerda
    TC_03 = [0, 0, 1, +C/2; 0, 1, 0, -L/2; -1, 0, 0, 0; 0, 0, 0, 1]; % centro -> perna dianteira/direita
    TC_04 = [0, 0, 1, -C/2; 0, 1, 0, -L/2; -1, 0, 0, 0; 0, 0, 0, 1]; % centro -> perna traseira/direita

    % Transformação homogênea plataforma-pata (posição final da perna)
    T0N_leg1 = left_legs.fkine(q_leg1).T
    T0N_leg2 = left_legs.fkine(q_leg2).T;
    T0N_leg3 = right_legs.fkine(q_leg3).T;
    T0N_leg4 = right_legs.fkine(q_leg4).T;

    % Transformação homogênea de cada link
    T_links_leg1 = {left_legs.A(1, q_leg1).T, left_legs.A(2, q_leg1).T, left_legs.A(3, q_leg1).T};
    T_links_leg2 = {left_legs.A(1, q_leg2).T, left_legs.A(2, q_leg2).T, left_legs.A(3, q_leg2).T};
    T_links_leg3 = {right_legs.A(1, q_leg3).T, right_legs.A(2, q_leg3).T, right_legs.A(3, q_leg3).T};
    T_links_leg4 = {right_legs.A(1, q_leg4).T, right_legs.A(2, q_leg4).T, right_legs.A(3, q_leg4).T}; 
    
    display("Transformações homogêneas da Perna")
    T01 = left_legs.A(1, q_leg1).T;
    T12 = left_legs.A(2, q_leg1).T;
    T23 = left_legs.A(3, q_leg1).T;
    
    % Plot da cinemática direta do robô
    figure;
    plot_frames_pos(TIC, TC_01, T_links_leg1{1}, T_links_leg1{2}, T_links_leg1{3});
    plot_frames_pos(TIC, TC_02, T_links_leg2{1}, T_links_leg2{2}, T_links_leg2{3});
    plot_frames_pos(TIC, TC_03, T_links_leg3{1}, T_links_leg3{2}, T_links_leg3{3});
    plot_frames_pos(TIC, TC_04, T_links_leg4{1}, T_links_leg4{2}, T_links_leg4{3});
    
    % Coordenadas dos pontos finais
    pos_1 = transl(TIC*TC_01)';
    pos_2 = transl(TIC*TC_02)';
    pos_3 = transl(TIC*TC_03)';
    pos_4 = transl(TIC*TC_04)';
    x = [pos_1(1), pos_3(1), pos_4(1), pos_2(1), pos_1(1)];
    y = [pos_1(2), pos_3(2), pos_4(2), pos_2(2), pos_1(2)];
    z = [pos_1(3), pos_3(3), pos_4(3), pos_2(3), pos_1(3)];

    % Plotar a linha conectando os pontos
    plot3(x, y, z, '-o', 'LineWidth', 2, 'Color', 'b');

    % Cálculo da posição final para cada perna
    pos_leg_1 = h2e((TIC * TC_01 * T0N_leg1) * [0; 0; 0; 1]);
    pos_leg_2 = h2e((TIC * TC_02 * T0N_leg2) * [0; 0; 0; 1]);
    pos_leg_3 = h2e((TIC * TC_03 * T0N_leg3) * [0; 0; 0; 1]);
    pos_leg_4 = h2e((TIC * TC_04 * T0N_leg4) * [0; 0; 0; 1]);
    
end