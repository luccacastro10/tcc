function trajetoria_global_ciclos = generate_legs_traj(p_leg1, p_leg2, p_leg3, p_leg4, a, b, duracao, duracao_pausa, taxa_amostragem, ciclos)

pontos_por_perna = duracao * taxa_amostragem;
one = ones(pontos_por_perna, 1);
pos0_leg1_mat = [p_leg1(1)*one, p_leg1(2)*one, p_leg1(3)*one];
pos0_leg2_mat = [p_leg2(1)*one, p_leg2(2)*one, p_leg2(3)*one];
pos0_leg3_mat = [p_leg3(1)*one, p_leg3(2)*one, p_leg3(3)*one];
pos0_leg4_mat = [p_leg4(1)*one, p_leg4(2)*one, p_leg4(3)*one];
traj_pausa = [pos0_leg1_mat, pos0_leg2_mat, pos0_leg3_mat, pos0_leg4_mat];
traj_pausa = traj_pausa(1:duracao_pausa*taxa_amostragem, :);

passo_leg1 = generate_elipse_traj(p_leg1, a, b, duracao, taxa_amostragem);
suporte_leg1 = generate_elipse_traj(p_leg1, a, b, duracao, taxa_amostragem, false);
passo_leg2 = generate_elipse_traj(p_leg2, a, b, duracao, taxa_amostragem);
suporte_leg2 = generate_elipse_traj(p_leg2, a, b, duracao, taxa_amostragem, false);
passo_leg3 = generate_elipse_traj(p_leg3, a, b, duracao, taxa_amostragem);
suporte_leg3 = generate_elipse_traj(p_leg3, a, b, duracao, taxa_amostragem, false);
passo_leg4 = generate_elipse_traj(p_leg4, a, b, duracao, taxa_amostragem);
suporte_leg4 = generate_elipse_traj(p_leg4, a, b, duracao, taxa_amostragem, false);

traj_leg_1 = [passo_leg1, suporte_leg2, suporte_leg3, passo_leg4];
traj_leg_2 = [suporte_leg1, passo_leg2, passo_leg3, suporte_leg4];
traj_leg_3 = [suporte_leg1, passo_leg2, passo_leg3, suporte_leg4];
traj_leg_4 = [passo_leg1, suporte_leg2, suporte_leg3, passo_leg4];

% perna 1 -> perna 4 -> perna 3 -> perna 2
trajetoria_global = [traj_pausa; traj_leg_1; traj_pausa; traj_leg_2; traj_pausa; traj_leg_1; traj_pausa; traj_leg_2];

x = passo_leg1(:,1);
y = passo_leg1(:,2); % constante
z = passo_leg1(:,3);
% Criar figura
figure;
hold on;
axis equal;
grid on;
xlabel('X');
ylabel('Z');
title('Animação da trajetória elíptica no plano XZ');
xlim([min(x)-0.2, max(x)+0.2]);
ylim([min(z)-0.2, max(z)+0.2]);
% Plot da trajetória completa (linha de fundo)
plot(x, z, '--', 'Color', [0.7 0.7 0.7]);
% Objeto para ponto atual
traj_point = plot(x(1), z(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% Animação
for k = 1:length(x)
    set(traj_point, 'XData', x(k), 'YData', z(k));
    pause(1 / taxa_amostragem); % controle da velocidade da animação
end

tempo = linspace(0, ciclos*4*(duracao+duracao_pausa), ciclos*round(4*(duracao+duracao_pausa) * taxa_amostragem))'; % para dar 4 passos

trajetoria_global_ciclos = []; % Ou trajetoria_global = trajetoria_original;
trajetoria_global_ciclos = repmat(trajetoria_global, ciclos, 1);
trajetoria_global_ciclos = [tempo, trajetoria_global_ciclos];

end