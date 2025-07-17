function [trajetoria, derivada] = generate_semi_elipse_traj(pos_inicial, a, b, duracao, taxa_amostragem)
% GERA_SEMI_ELIPSE_TRAJ Gera trajetória de semi-elipse com retorno linear
%
% Inputs:
%   pos_inicial - ponto inicial [x; y; z] (3x1)
%   a - semi-eixo maior da elipse (em x)
%   b - semi-eixo menor da elipse (em z)
%   duracao - tempo total da trajetória (segundos)
%   taxa_amostragem - pontos por segundo (Hz)
%
% Outputs:
%   trajetoria - matriz Nx3 com [x, y, z]
%   derivada   - matriz Nx3 com [vx, vy, vz]

% Calcular número de pontos (metade para cada parte)
num_pontos = round(duracao * taxa_amostragem);
num_pontos_semi = ceil(num_pontos/2);
num_pontos_reta = num_pontos - num_pontos_semi;

% Extrair coordenadas do ponto inicial
x0 = pos_inicial(1);
y0 = pos_inicial(2);
z0 = pos_inicial(3);

%% Parte 1: Semi-elipse superior (0 a π)
theta = linspace(0, pi, num_pontos_semi);

% Trajetória elíptica (parte superior)
x_elipse = x0 + a * -cos(theta) + a;
z_elipse = z0 + b * sin(theta);
y_elipse = y0 * ones(1, num_pontos_semi);

% Derivada da elipse
dtheta_dt = pi / (duracao/2); % Metade do tempo para a semi-elipse
vx_elipse = a * sin(theta) * dtheta_dt;
vz_elipse = b * cos(theta) * dtheta_dt;
vy_elipse = zeros(1, num_pontos_semi);

%% Parte 2: Retorno linear (π a 2π)
% Pontos de início e fim do retorno
ponto_final = [x0 + a * -cos(pi) + a, y0, z0 + b * sin(pi)]; % Ponto final da semi-elipse
ponto_inicial = [x0 + a * -cos(0) + a, y0, z0 + b * sin(0)]; % Ponto inicial

% Trajetória linear
t_reta = linspace(0, 1, num_pontos_reta);
x_reta = ponto_final(1) + (ponto_inicial(1) - ponto_final(1)) * t_reta;
z_reta = ponto_final(3) + (ponto_inicial(3) - ponto_final(3)) * t_reta;
y_reta = y0 * ones(1, num_pontos_reta);

% Derivada da reta (velocidade constante)
vel_reta = (ponto_inicial - ponto_final) / (duracao/2); % Metade do tempo para o retorno
vx_reta = vel_reta(1) * ones(1, num_pontos_reta);
vz_reta = vel_reta(3) * ones(1, num_pontos_reta);
vy_reta = zeros(1, num_pontos_reta);

%% Combinar ambas as partes
trajetoria = [
    [x_elipse; y_elipse; z_elipse]', 
    [x_reta; y_reta; z_reta]'
];

derivada = [
    [vx_elipse; vy_elipse; vz_elipse]',
    [vx_reta; vy_reta; vz_reta]'
];

% Garantir que termina exatamente no ponto inicial
trajetoria(end,:) = pos_inicial';
end