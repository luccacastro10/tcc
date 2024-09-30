
% Posição do centro de massa do robô
cx = 10;
cy = 15;
cz = 20;
pos = [cx, cy, cz];

% Angulação do centro de massa do robô
roll  = 0;
pitch = 0;
yaw   = 0;
rpy = [roll, pitch, yaw];

% Configuração das juntas da perna
q_leg1 = [0, pi/4, -pi/2];
q_leg2 = [0, pi/4, -pi/2];
q_leg3 = [0, pi/4, -pi/2];
q_leg4 = [0, pi/4, -pi/2];

% Cálculo da Cinemática Direta das 4 patas do robô
[pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4] = direct_cinematics(pos, rpy, q_leg1, q_leg2, q_leg3, q_leg4);