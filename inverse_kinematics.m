function [q_leg1, q_leg2, q_leg3, q_leg4] = inverse_kinematics(pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4)
%INVERSE_KINEMATICS Calcula a cinemática inversa das quatro pernas do robô
%
%   [q_leg1, q_leg2, q_leg3, q_leg4] = inverse_kinematics(pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4)
%
%   Resolve os ângulos articulares das quatro pernas do robô a partir das
%   posições finais desejadas de cada perna no espaço cartesiano, com relação ao seu centro de massa.
%
%   Entradas:
%       pos_leg_N - vetor (3x1) posição desejada da perna N
%
%   Saídas:
%       q_legN - vetor coluna (3x1) ângulos articulares da perna N
%
%   Observações:
%       - Os objetos SerialLink das pernas são carregados do arquivo 'constants.mat'.
%       - A função utiliza o método ikine da Robotics Toolbox para resolver a cinemática inversa.
%       - A máscara [1 1 1 0 0 0] indica que apenas a posição (x,y,z) é considerada, ignorando orientação.
%       - A estimativa inicial para as juntas é [0,0,0].
%
%   Exemplo de uso:
%       [q1, q2, q3, q4] = inverse_kinematics(pos1, pos2, pos3, pos4);
%
%   Autor: Lucca
%   Data: 2025-04-21

    load('constants.mat', 'leg1', 'leg2', 'leg3', 'leg4')   
    
    % Estimativa inicial para as configurações das juntas
    initial_guess = [0, 0, 0];

    % Resolução da cinemática inversa para a perna esquerda usando ikine    
    q_leg1 = leg1.ikine(transl(pos_leg_1), 'q0', initial_guess, 'mask', [1 1 1 0 0 0], 'ilimit', 10e4)';
    q_leg2 = leg2.ikine(transl(pos_leg_2), 'q0', initial_guess, 'mask', [1 1 1 0 0 0], 'ilimit', 10e4)';
    q_leg3 = leg3.ikine(transl(pos_leg_3), 'q0', initial_guess, 'mask', [1 1 1 0 0 0], 'ilimit', 10e4)';
    q_leg4 = leg4.ikine(transl(pos_leg_4), 'q0', initial_guess, 'mask', [1 1 1 0 0 0], 'ilimit', 10e4)';

end
