
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

leg1 = SerialLink(links);
leg2 = SerialLink(links);
leg1.name = "leg1";
leg2.name = "leg2";

% Pernas da direita
links(1) = Link([0, 0, 0, -pi/2], 'standard');
links(2) = Link([0, L1, -L2, 0], 'standard');
links(3) = Link([0, 0, -L3, 0], 'standard');

links(1).offset = pi;
links(1).qlim = link_1_limits;
links(2).qlim = link_2_limits;
links(3).qlim = link_3_limits;

leg3 = SerialLink(links);
leg4 = SerialLink(links);
leg3.name = "leg3";
leg4.name = "leg4";

% Transformações homogêneas do centro para os links iniciais de cada perna
leg1.base = [0, 0, 1, +C/2; 0, 1, 0, +L/2; -1, 0, 0, 0; 0, 0, 0, 1]; % centro -> perna dianteira/esquerda
leg2.base = [0, 0, 1, -C/2; 0, 1, 0, +L/2; -1, 0, 0, 0; 0, 0, 0, 1]; % centro -> perna traseira/esquerda
leg3.base = [0, 0, 1, +C/2; 0, 1, 0, -L/2; -1, 0, 0, 0; 0, 0, 0, 1]; % centro -> perna dianteira/direita
leg4.base = [0, 0, 1, -C/2; 0, 1, 0, -L/2; -1, 0, 0, 0; 0, 0, 0, 1]; % centro -> perna traseira/direita

save('constants.mat','C', 'L', 'L1', 'L2', 'L3', 'leg1', 'leg2', 'leg3', 'leg4')