clear all, clc
constants
step_constants

%TODO: Angle constraint no modelo simulink
%TODO: Avaliar como a transformação homogênea TIC será tratada posteriormente
%TODO: Ler C e L dinamicamente no matlab function block

% Configuração das juntas da perna
q_leg1 = [+pi/9; +pi/6; -pi/3];
q_leg2 = [+pi/9; +pi/6; -pi/3];
q_leg3 = [-pi/9; -pi/6; +pi/3];
q_leg4 = [-pi/9; -pi/6; +pi/3];

simulink_initial_q = [0;0;0];

% Cálculo da Cinemática Direta das 4 patas do robô
[pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4] = foward_kinematics(q_leg1, q_leg2, q_leg3, q_leg4);

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

last_roll = 0;
last_pitch = 0;
last_yaw = 0;
last_u_roll = 0;
last_u_pitch = 0;
last_u_yaw = 0;

initial_hight = transl(leg1.fkine(simulink_initial_q).T);
initial_hight_z = abs(initial_hight(3));