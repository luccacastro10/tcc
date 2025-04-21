% Carregar as constantes
load('constants.mat', 'leg1', 'leg2', 'leg3', 'leg4');

% Configuração das juntas da perna
q0_leg1 = [+pi/9; +pi/6; -pi/3];
q0_leg2 = [+pi/9; +pi/6; -pi/3];
q0_leg3 = [-pi/9; -pi/6; +pi/3];
q0_leg4 = [-pi/9; -pi/6; +pi/3];

pc_leg1_q0 = e2h(transl(leg1.fkine(q0_leg1).T));
pc_leg2_q0 = e2h(transl(leg2.fkine(q0_leg2).T));
pc_leg3_q0 = e2h(transl(leg3.fkine(q0_leg3).T));
pc_leg4_q0 = e2h(transl(leg4.fkine(q0_leg4).T));

wn = pi/2;          % Frequência angular (rad/s)

% Tempo para uma volta completa (2*pi/wn)
t = linspace(0, 2*pi/wn, 500);  % 100 pontos para uma volta completa

% Definir a altura do passo usando uma curva de Bézier quadrática
step_length = 50;
step_count = 10;
step_high = 20; % Altura máxima do passo
t_step = linspace(-sqrt(step_high), +sqrt(step_high), step_length);
step = step_high - t_step.^2; % Curva de Bézier quadrática
step_dot = -2*t_step;

i_step = 0;
current_leg = 1;


