% CONSTANTS.M
% Define parâmetros geométricos da plataforma e pernas do robô quadrúpede,
% configura os elos com parâmetros DH e limites articulares,
% cria objetos SerialLink para as quatro pernas,
% e define as transformações base de cada perna em relação ao centro do robô.
% Salva todas as variáveis em 'constants.mat' para uso posterior.

%joints:
joint_damping = 0.01;
ps_converter_time_constant = 1e-3;

%initial_conditions:
% com_initial_height = 0.43; % metros
com_initial_height = 0.35; % metros
% com_initial_height = 0.5; % metros

q0_leg1 = [+pi/9; +pi/6; -pi/3];
q0_leg2 = [+pi/9; +pi/6; -pi/3];
q0_leg3 = [-pi/9; -pi/6; +pi/3];
q0_leg4 = [-pi/9; -pi/6; +pi/3];
q0 = [q0_leg1; q0_leg2; q0_leg3; q0_leg4];

% world_condition:
world_damping = 0;      % Translational damping for 6-DOF joint [N/m]
world_rot_damping = 0;  % Rotational damping for 6-DOF joint [N*m/(rad/s)]

% Parâmetros físicos do robô
C = 80;  % Comprimento
L = 30;  % Largura
H = 3;   % Altura
L1 = 10; % Comprimento elo 1 da perna
L2 = 20; % Comprimento elo 2 da perna
L3 = 20; % Comprimento elo 3 da perna
density = 1000;
weight = density*(C*L*H+H*H*(L1+L2+L3)+(4/3)*pi*(H^3))*(1e-6)*9.8;

% Contact and friction parameters:
contact_stiffness = weight/0.001;        % Approximated at weight (N) / desired displacement (m)
contact_damping = contact_stiffness/10; % Tuned based on contact stiffness value
mu_s = 1.2;     % Static friction coefficient: Around that of rubber-asphalt
mu_k = 1;     % Kinetic friction coefficient: Lower than the static coefficient
mu_vth = 0.01;   % Friction velocity threshold (m/s)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% ##### Definição dos parâmetros DH (theta, d, a, alpha) #####

link_1_limits = [-pi/6, pi/6];
link_2_limits = [-2*pi/6, 2*pi/6];
link_3_limits = [-pi, 0];

% Pernas da esquerda
left_links(1) = Link([0, 0, 0, pi/2], 'standard');
left_links(2) = Link([0, L1, -L2, 0], 'standard');
left_links(3) = Link([0, 0, -L3, 0], 'standard');

left_links(1).offset = pi;
left_links(1).qlim = link_1_limits;
left_links(2).qlim = link_2_limits;
left_links(3).qlim = link_3_limits;

leg1 = SerialLink(left_links);
leg2 = SerialLink(left_links);
leg1.name = "leg1";
leg2.name = "leg2";

% Pernas da direita
right_links(1) = Link([0, 0, 0, -pi/2], 'standard');
right_links(2) = Link([0, L1, -L2, 0], 'standard');
right_links(3) = Link([0, 0, -L3, 0], 'standard');

right_links(1).offset = pi;
right_links(1).qlim = link_1_limits;
right_links(2).qlim = link_2_limits;
right_links(3).qlim = link_3_limits;

leg3 = SerialLink(right_links);
leg4 = SerialLink(right_links);
leg3.name = "leg3";
leg4.name = "leg4";

% Transformações homogêneas do centro para os links iniciais de cada perna
leg1.base = [0, 0, 1, +C/2; 0, 1, 0, +L/2; -1, 0, 0, 0; 0, 0, 0, 1]; % centro -> perna dianteira/esquerda
leg2.base = [0, 0, 1, -C/2; 0, 1, 0, +L/2; -1, 0, 0, 0; 0, 0, 0, 1]; % centro -> perna traseira/esquerda
leg3.base = [0, 0, 1, +C/2; 0, 1, 0, -L/2; -1, 0, 0, 0; 0, 0, 0, 1]; % centro -> perna dianteira/direita
leg4.base = [0, 0, 1, -C/2; 0, 1, 0, -L/2; -1, 0, 0, 0; 0, 0, 0, 1]; % centro -> perna traseira/direita
pc01 = leg1.base.T; pc02 = leg2.base.T; pc03 = leg3.base.T; pc04 = leg4.base.T; % usado no bloco recalculate_legs_position
P0s = [transl(pc01)'; transl(pc02)'; transl(pc03)'; transl(pc04)']; % usado no bloco inverse_kinematics_analytical

% pseudoInverse control K
k = 300;
Kvec = k*ones(12,1);

save('constants.mat','C', 'L', 'L1', 'L2', 'L3', 'leg1', 'leg2', 'leg3', 'leg4')