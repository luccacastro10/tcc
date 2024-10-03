
% Parâmetros da plataforma do robô
C = 80; % Comprimento
L = 30; % Largura
H = 3; % Altura

% Parâmetros das pernas do robô
L1 = 10; % Comprimento elo 1 da perna
L2 = 20; % Comprimento elo 2 da perna
L3 = 20; % Comprimento elo 3 da perna

% ##### Definição dos parâmetros DH (theta, d, a, alpha) #####

link_1_limits = [-pi/6, pi/6];
link_2_limits = [-2*pi/6, 2*pi/6];
link_3_limits = [-pi, 0];

% Pernas da esquerda
links(1) = Link([0, 0, 0, pi/2], 'standard');
links(2) = Link([0, L1, -L2, 0], 'standard');
links(3) = Link([0, 0, -L3, 0], 'standard');

links(1).offset = pi;
links(1).qlim = link_1_limits;
links(2).qlim = link_2_limits;
links(3).qlim = link_3_limits;

left_legs = SerialLink(links);
left_legs.name = "left_legs";

% Pernas da direita
links(1) = Link([0, 0, 0, -pi/2], 'standard');
links(2) = Link([0, L1, -L2, 0], 'standard');
links(3) = Link([0, 0, -L3, 0], 'standard');

links(1).offset = pi;
links(1).qlim = link_1_limits;
links(2).qlim = link_2_limits;
links(3).qlim = link_3_limits;

right_legs = SerialLink(links);
right_legs.name = "right_legs";

save('constants.mat','C', 'L', 'L1', 'L2', 'L3', 'left_legs', 'right_legs')