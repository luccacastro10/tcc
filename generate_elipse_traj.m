function [trajetoria] = generate_elipse_traj(pos_inicial, a, b, duracao, taxa_amostragem, perna_principal)
% GERA_TRAJETORIA_ELIPSE Gera trajetória elíptica no plano XZ com uma única volta
% e calcula sua derivada (velocidade)
%
% Inputs:
%   pos_inicial - ponto inicial [x; y; z] (3x1)
%   a - semi-eixo maior da elipse (em x)
%   b - semi-eixo menor da elipse (em z)
%   duracao - tempo total para completar uma volta (segundos)
%   taxa_amostragem - pontos por segundo (Hz)
%   perna_principal - se true, faz elipse; se false, faz linha vai-e-volta
%
% Outputs:
%   trajetoria - matriz Nx3 com [x, y, z]
%   derivada   - matriz Nx3 com [vx, vy, vz]

if nargin < 6
    perna_principal = true; % Valor padrão
end

% Calcular número de pontos
num_pontos = round(duracao * taxa_amostragem);

% Extrair coordenadas do ponto inicial
x0 = pos_inicial(1);
y0 = pos_inicial(2);
z0 = pos_inicial(3);

if perna_principal
    % === TRAJETÓRIA ELÍPTICA ===
    theta = linspace(0, pi, (num_pontos)/2 + 1);
    theta(end) = [];
    x = x0 + a * +cos(theta) + a;
    z = z0 + b * sin(theta);
    y = y0 * ones(1, (num_pontos)/2);

else
    % === TRAJETÓRIA LINEAR (VAI E VOLTA) ===

    % Metade dos pontos para ida, metade para volta
    n_ida = 3*floor(num_pontos / 2);
    
    % Movimento linear no eixo X
    x   = linspace(x0-2*a, x0, n_ida);

    % Constantes nos outros eixos
    y = y0 * ones(1, n_ida);
    z = z0 * ones(1, n_ida);

end

% Formatar saída
trajetoria = [x; y; z]';
trajetoria = flipud(trajetoria);

end
