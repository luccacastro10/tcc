constants

%TODO: Angle constraint no modelo simulink
%TODO: Avaliar como a transformação homogênea TIC será tratada posteriormente
%TODO: Ler C e L dinamicamente no matlab function block

% Cálculo da Cinemática Direta das 4 patas do robô
[pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4] = foward_kinematics(q0_leg1, q0_leg2, q0_leg3, q0_leg4);

% Cálculo da Cinemática Inversa das 4 patas do robô
% [new_q_leg1, new_q_leg2, new_q_leg3, new_q_leg4] = inverse_kinematics(pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4);
[new_q_leg1, new_q_leg2, new_q_leg3, new_q_leg4] = literal_inverse_kinematics(pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4);

% Validação da Cinemática Inversa
[new_pos_leg_1, new_pos_leg_2, new_pos_leg_3, new_pos_leg_4] = foward_kinematics(new_q_leg1, new_q_leg2, new_q_leg3, new_q_leg4);

% Validação da Cinemática Direta com base nos cálculos manuais
% pos_leg_1;
% leg1.jacob0(q_leg1);
% [pos_leg_1_x_calc, pos_leg_1_y_calc, pos_leg_1_z_calc, J_leg_1_calc] = literal_kinematics(leg1, q_leg1);
% 
% pos_leg_1
% new_pos_leg_1