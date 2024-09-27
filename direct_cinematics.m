function [pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4] = direct_cinematics(pos, rpy, q)
    
    load('constanst.mat','C', 'L', 'L1', 'L2', 'L3')   
    
    % Definição dos parâmetros DH (theta, d, a, alpha)
    l(1) = Link([0, 0, 0, pi/2], 'standard');
    l(2) = Link([0, L1, -L2, 0], 'standard');
    l(3) = Link([0, 0, -L3, 0], 'standard');

    l(1).offset = pi/2;

    % Definição dos limites das juntas
    l(1).qlim = [-pi/6, pi/6];
    l(2).qlim = [-2*pi/6, 2*pi/6];
    l(3).qlim = [-pi, 0];

    % Criação do robô
    leg = SerialLink(l);
    leg.name = "Leg";
    leg.teach

    % Transformação homogênea inercial-centro
    TIC = transl(pos) * rpy2tr(rpy);

    % Transformações homogêneas do centro para os links iniciais de cada perna
    TC_01 = [0, 0, 1, +C/2; +1, 0, 0, +L/2; 0, 1, 0, 0; 0, 0, 0, 1]; % centro -> perna dianteira/esquerda
    TC_02 = [0, 0, 1, -C/2; +1, 0, 0, +L/2; 0, 1, 0, 0; 0, 0, 0, 1]; % centro -> perna traseira/esquerda
    TC_03 = [0, 0, 1, +C/2; -1, 0, 0, -L/2; 0, 1, 0, 0; 0, 0, 0, 1]; % centro -> perna dianteira/direita
    TC_04 = [0, 0, 1, -C/2; -1, 0, 0, -L/2; 0, 1, 0, 0; 0, 0, 0, 1]; % centro -> perna traseira/direita

    % Transformação homogênea plataforma-pata (posição final da perna)
    T0N = leg.fkine(q).T;

    % Transformação homogênea de cada link
    T0_1 = leg.A(1, q).T;
    T1_2 = leg.A(2, q).T;
    T2_3 = leg.A(3, q).T;
    
    
    plot_frames_pos(TIC, TC_01, T0_1, T1_2, T2_3, false);
    plot_frames_pos(TIC, TC_02, T0_1, T1_2, T2_3, true);
    plot_frames_pos(TIC, TC_03, T0_1, T1_2, T2_3, true);
    plot_frames_pos(TIC, TC_04, T0_1, T1_2, T2_3, true);

    % animate_leg_frames(TIC, TC_01, T0_1, T1_2, T2_3)

    % Cálculo da posição final para cada perna
    pos_leg_1 = h2e((TIC * TC_01 * T0N) * [0; 0; 0; 1]);
    pos_leg_2 = h2e((TIC * TC_02 * T0N) * [0; 0; 0; 1]);
    pos_leg_3 = h2e((TIC * TC_03 * T0N) * [0; 0; 0; 1]);
    pos_leg_4 = h2e((TIC * TC_04 * T0N) * [0; 0; 0; 1]);
    
end