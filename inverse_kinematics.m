function [q_leg1, q_leg2, q_leg3, q_leg4] = inverse_kinematics(pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4)

    load('constants.mat', 'left_legs', 'right_legs', 'TC_01', 'TC_02', 'TC_03', 'TC_04')     

    % Matriz de transformação homogênea para a pose de destino (somente posição)
    T_leg1 = transl(t2r(TC_01)\(pos_leg_1 - TC_01(1:3, 4)));
    T_leg2 = transl(t2r(TC_02)\(pos_leg_2 - TC_02(1:3, 4)));
    T_leg3 = transl(t2r(TC_03)\(pos_leg_3 - TC_03(1:3, 4)));
    T_leg4 = transl(t2r(TC_04)\(pos_leg_4 - TC_04(1:3, 4)));
    
    % Estimativa inicial para as configurações das juntas
    initial_guess = [0, 0, 0];

    % Resolução da cinemática inversa para a perna esquerda usando ikine
    q_leg1 = left_legs.ikine(transl(T_leg1(1:3,4)), 'q0', initial_guess, 'mask', [1 1 1 0 0 0]);
    q_leg2 = left_legs.ikine(transl(T_leg2(1:3,4)), 'q0', initial_guess, 'mask', [1 1 1 0 0 0]);
    q_leg3 = right_legs.ikine(transl(T_leg3(1:3,4)), 'q0', initial_guess, 'mask', [1 1 1 0 0 0]);
    q_leg4 = right_legs.ikine(transl(T_leg4(1:3,4)), 'q0', initial_guess, 'mask', [1 1 1 0 0 0]);

end
