function [pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4] = foward_kinematics(pos, rpy, q_leg1, q_leg2, q_leg3, q_leg4)
    
    load('constanst.mat','C', 'L', 'L1', 'L2', 'L3')   
    
    % Definição dos parâmetros DH (theta, d, a, alpha)
    % TODO: ENTENDER SE O SINAL DE Ai DEVE SER NEGATIVO MESMO!!!
    l(1) = Link([0, 0, 0, pi/2], 'standard');
    l(2) = Link([0, L1, L2, 0], 'standard');
    l(3) = Link([0, 0, L3, 0], 'standard');

    l(1).offset = pi/2;

    % Definição dos limites das juntas
    l(1).qlim = [-pi/6, pi/6];
    l(2).qlim = [-2*pi/6, 2*pi/6];
    l(3).qlim = [-pi, 0];

    % Criação do robô
    leg = SerialLink(l);
    leg.name = "Leg";
    leg.teach
    %leg.plot(q_leg1);
    J = leg.jacob0(q_leg1);

    % Transformação homogênea inercial-centro
    TIC = transl(pos) * rpy2tr(rpy);

    % Transformações homogêneas do centro para os links iniciais de cada perna
    TC_01 = [0, 0, 1, +C/2; +1, 0, 0, +L/2; 0, 1, 0, 0; 0, 0, 0, 1]; % centro -> perna dianteira/esquerda
    TC_02 = [0, 0, 1, -C/2; +1, 0, 0, +L/2; 0, 1, 0, 0; 0, 0, 0, 1]; % centro -> perna traseira/esquerda
    TC_03 = [0, 0, 1, +C/2; -1, 0, 0, -L/2; 0, 1, 0, 0; 0, 0, 0, 1]; % centro -> perna dianteira/direita
    TC_04 = [0, 0, 1, -C/2; -1, 0, 0, -L/2; 0, 1, 0, 0; 0, 0, 0, 1]; % centro -> perna traseira/direita

    % Transformação homogênea plataforma-pata (posição final da perna)
    T0N_leg1 = leg.fkine(q_leg1).T;
    T0N_leg2 = leg.fkine(q_leg2).T;
    T0N_leg3 = leg.fkine(q_leg3).T;
    T0N_leg4 = leg.fkine(q_leg4).T;

    % Transformação homogênea de cada link
    T_links_leg1 = {leg.A(1, q_leg1).T, leg.A(2, q_leg1).T, leg.A(3, q_leg1).T};
    T_links_leg2 = {leg.A(1, q_leg2).T, leg.A(2, q_leg2).T, leg.A(3, q_leg2).T};
    T_links_leg3 = {leg.A(1, q_leg3).T, leg.A(2, q_leg3).T, leg.A(3, q_leg3).T};
    T_links_leg4 = {leg.A(1, q_leg4).T, leg.A(2, q_leg4).T, leg.A(3, q_leg4).T}; 
    
    display("Transformações homogêneas da Perna")
    T01 = leg.A(1, q_leg1).T
    T12 = leg.A(2, q_leg1).T
    T23 = leg.A(3, q_leg1).T
    
    %syms q1, q2, q3;
    %T03 = trchain( , [q1 q2 q3])
    %T03 = simplify(T03);
    %p=T03(1:3,4);
    
    hold off;
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

    %animate_leg_frames(TIC, TC_01, T_links_leg1{1}, T_links_leg1{2}, T_links_leg1{3})

    % Cálculo da posição final para cada perna
    pos_leg_1 = h2e((TIC * TC_01 * T0N_leg1) * [0; 0; 0; 1]);
    pos_leg_2 = h2e((TIC * TC_02 * T0N_leg2) * [0; 0; 0; 1]);
    pos_leg_3 = h2e((TIC * TC_03 * T0N_leg3) * [0; 0; 0; 1]);
    pos_leg_4 = h2e((TIC * TC_04 * T0N_leg4) * [0; 0; 0; 1]);
    
end