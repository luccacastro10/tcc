function trajetoria_global_ciclos = generate_legs_traj(p_leg1, p_leg2, p_leg3, p_leg4, a, b, duracao, duracao_pausa, taxa_amostragem, ciclos)

ida_leg1 = generate_elipse_traj(p_leg1, a, b, duracao, taxa_amostragem);
ida_leg2 = generate_elipse_traj(p_leg2, a, b, duracao, taxa_amostragem);
ida_leg3 = generate_elipse_traj(p_leg3, a, b, duracao, taxa_amostragem);
ida_leg4 = generate_elipse_traj(p_leg4, a, b, duracao, taxa_amostragem);

volta_leg1 = generate_elipse_traj(p_leg1, a, b, duracao, taxa_amostragem, false);
volta_leg2 = generate_elipse_traj(p_leg2, a, b, duracao, taxa_amostragem, false);
volta_leg3 = generate_elipse_traj(p_leg3, a, b, duracao, taxa_amostragem, false);
volta_leg4 = generate_elipse_traj(p_leg4, a, b, duracao, taxa_amostragem, false);

n_volta = length(volta_leg1);
terco = n_volta / 3;
v1_leg1 = volta_leg1(1         : terco,:);
v2_leg1 = volta_leg1(terco + 1 : 2*terco,:);
v3_leg1 = volta_leg1(2*terco + 1 : end,:);
v1_leg2 = volta_leg2(1         : terco,:);
v2_leg2 = volta_leg2(terco + 1 : 2*terco,:);
v3_leg2 = volta_leg2(2*terco + 1 : end,:);
v1_leg3 = volta_leg3(1         : terco,:);
v2_leg3 = volta_leg3(terco + 1 : 2*terco,:);
v3_leg3 = volta_leg3(2*terco + 1 : end,:);
v1_leg4 = volta_leg4(1         : terco,:);
v2_leg4 = volta_leg4(terco + 1 : 2*terco,:);
v3_leg4 = volta_leg4(2*terco + 1 : end,:);

traj_1 = [ida_leg1, v1_leg2,  v2_leg3, v3_leg4];
traj_2 = [v1_leg1,  v2_leg2,  v3_leg3, ida_leg4];
traj_3 = [v2_leg1,  v3_leg2, ida_leg3, v1_leg4];
traj_4 = [v3_leg1, ida_leg2,  v1_leg3, v2_leg4];

pontos_por_perna = duracao_pausa * taxa_amostragem;
one = ones(pontos_por_perna, 1);
pos0_leg1_mat = [ida_leg1(1,1)*one, ida_leg1(1,2)*one, ida_leg1(1,3)*one];
pos0_leg2_mat = [v1_leg2(1,1)*one, v1_leg2(1,2)*one, v1_leg2(1,3)*one];
pos0_leg3_mat = [v2_leg3(1,1)*one, v2_leg3(1,2)*one, v2_leg3(1,3)*one];
pos0_leg4_mat = [v3_leg4(1,1)*one, v3_leg4(1,2)*one, v3_leg4(1,3)*one];
traj_pausa = [pos0_leg1_mat, pos0_leg2_mat, pos0_leg3_mat, pos0_leg4_mat];
traj_pausa = traj_pausa(1:duracao_pausa*taxa_amostragem, :);

% perna 1 -> perna 4 -> perna 3 -> perna 2
trajetoria_global = [traj_1; traj_2; traj_3; traj_4];

x = [ida_leg4(:,1); v1_leg4(:,1); v2_leg4(:,1); v3_leg4(:,1)];
y = [ida_leg4(:,2); v1_leg4(:,2); v2_leg4(:,2); v3_leg4(:,2)];
z = [ida_leg4(:,3); v1_leg4(:,3); v2_leg4(:,3); v3_leg4(:,3)];
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

tempo = linspace(0, duracao_pausa+ciclos*4*(duracao/2), duracao_pausa* taxa_amostragem+ciclos*round(4*(duracao/2) * taxa_amostragem))'; % para dar 4 passos

trajetoria_global_ciclos = []; % Ou trajetoria_global = trajetoria_original;
trajetoria_global_ciclos = repmat(trajetoria_global, ciclos, 1);
trajetoria_global_ciclos = [traj_pausa; trajetoria_global_ciclos];
trajetoria_global_ciclos = [tempo, trajetoria_global_ciclos];

end