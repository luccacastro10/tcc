function [q_leg1, q_leg2, q_leg3, q_leg4] = inverse_kinematics(pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4)

    load('constants.mat', 'leg1', 'leg2', 'leg3', 'leg4')   
    
    % Estimativa inicial para as configurações das juntas
    initial_guess = [0, 0, 0];

    % Resolução da cinemática inversa para a perna esquerda usando ikine    
    q_leg1 = leg1.ikine(transl(pos_leg_1), 'q0', initial_guess, 'mask', [1 1 1 0 0 0], 'ilimit', 10e4)';
    q_leg2 = leg2.ikine(transl(pos_leg_2), 'q0', initial_guess, 'mask', [1 1 1 0 0 0], 'ilimit', 10e4)';
    q_leg3 = leg3.ikine(transl(pos_leg_3), 'q0', initial_guess, 'mask', [1 1 1 0 0 0], 'ilimit', 10e4)';
    q_leg4 = leg4.ikine(transl(pos_leg_4), 'q0', initial_guess, 'mask', [1 1 1 0 0 0], 'ilimit', 10e4)';

end
