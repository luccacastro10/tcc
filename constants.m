
% Parâmetros da perna do robô
C = 12; % Comprimento
L = 5;  % Largura
L1 = 1; % Comprimento elo 1 da perna
L2 = 2; % Comprimento elo 2 da perna
L3 = 2; % Comprimento elo 3 da perna

% Definição dos parâmetros DH (theta, d, a, alpha)
% TODO: ENTENDER SE O SINAL DE Ai DEVE SER NEGATIVO MESMO!!!

% Pernas da esquerda ########################################
links(1) = Link([0, 0, 0, pi/2], 'standard');
links(2) = Link([0, L1, -L2, 0], 'standard');
links(3) = Link([0, 0, -L3, 0], 'standard');

links(1).offset = pi;

links(1).qlim = [-pi/6, pi/6];
links(2).qlim = [-2*pi/6, 2*pi/6];
links(3).qlim = [-pi, 0];

left_legs = SerialLink(links);
left_legs.name = "left_legs";

% Pernas da direita ########################################
links(1) = Link([0, 0, 0, -pi/2], 'standard');
links(2) = Link([0, L1, -L2, 0], 'standard');
links(3) = Link([0, 0, -L3, 0], 'standard');

links(1).offset = pi;

links(1).qlim = [-pi/6, pi/6];
links(2).qlim = [-2*pi/6, 2*pi/6];
links(3).qlim = [-pi, 0];

right_legs = SerialLink(links);
right_legs.name = "right_legs";

save('constants.mat','C', 'L', 'L1', 'L2', 'L3', 'left_legs', 'right_legs')