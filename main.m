
% Posição do centro de massa do robô
cx = 0;
cy = 0;
cz = 0;
pos = [cx, cy, cz];

% Angulação do centro de massa do robô
roll  = 0;
pitch = 0;
yaw   = 0;
rpy = [roll, pitch, yaw];

% Configuração das juntas da perna
q_leg1 = [0, pi/6, 0]
q_leg2 = [0, 0, pi/6]
q_leg3 = [0, 0, 0]
q_leg4 = [0, pi/3, pi/3]

% Cálculo da Cinemática Direta das 4 patas do robô
[pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4] = foward_kinematics(pos, rpy, q_leg1, q_leg2, q_leg3, q_leg4)

[new_q_leg1, new_q_leg2, new_q_leg3, new_q_leg4] = inverse_kinematics(pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4)

[new_pos_leg_1, new_pos_leg_2, new_pos_leg_3, new_pos_leg_4] = foward_kinematics(pos, rpy, new_q_leg1, new_q_leg2, new_q_leg3, new_q_leg4)

